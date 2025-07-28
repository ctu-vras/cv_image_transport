// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport codec working with CV format.
 * \author Martin Pecka
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <boost/format/format_fwd.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#if CV_VERSION_MAJOR < 4
#include <opencv2/imgcodecs/imgcodecs_c.h>
#endif
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <cv_image_transport/cv_codec.h>
#include <cv_image_transport/CVPublisherConfig.h>
#include <cv_image_transport/parse_compressed_format.h>

namespace cv_image_transport
{

namespace enc = sensor_msgs::image_encodings;

struct CVCodecPrivate
{
};

CVCodec::CVCodec(const ::cras::LogHelperPtr& logHelper) : ImageTransportCodec(logHelper),
  data(new CVCodecPrivate)
{
}

CVCodec::~CVCodec() = default;

CVCodec::DecodeResult CVCodec::decode(const sensor_msgs::CompressedImage& compressed) const
{
  const auto formatStruct = parseCompressedTransportFormat(compressed.format);
  if (!formatStruct.has_value())
    return cras::make_unexpected(formatStruct.error());

  const auto cvImage = cv_bridge::toCvCopy(compressed, formatStruct->rawEncoding);
  if (cvImage == nullptr)
    return cras::make_unexpected(formatStruct->formatString + " decoding failed.");

  sensor_msgs::Image image;
  cvImage->toImageMsg(image);

  if (image.width > 0 && image.height > 0)
    return image;

  return cras::make_unexpected("Decoding compressed image yielded a zero-size result.");
}

std::string CVCodec::getTransportName() const
{
  return "cv";
}

CVCodec::EncodeResult CVCodec::encode(
  const sensor_msgs::Image& raw, const std::string& format, const std::vector<int32_t>& cvOpts) const
{
  const auto formatStruct = extractCompressedTransportFormat(raw, format);
  if (!formatStruct.has_value())
    return cras::make_unexpected("Invalid compressed encoder config: " + formatStruct.error());

  const auto formatValidation = validateImageFormat(*formatStruct);
  if (!formatValidation.has_value())
    return cras::make_unexpected(formatValidation.error());

  sensor_msgs::CompressedImage compressed;
  compressed.header = raw.header;
  compressed.format = makeCompressedTransportFormat(formatStruct.value());

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(raw, nullptr, formatStruct->compressedEncoding);
  }
  catch (cv_bridge::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "cv_bridge error occurred while converting %ix%i %s image as %s: %s.",
      raw.width, raw.height, raw.encoding.c_str(), format.c_str(), e.what()));
  }
  catch (cv::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "OpenCV error occurred while converting %ix%i %s image as %s: %s (%s).",
      raw.width, raw.height, raw.encoding.c_str(), format.c_str(), cvErrorStr(e.code), e.err.c_str()));
  }

  try
  {
    const std::vector<int> opts(cvOpts.begin(), cvOpts.end());  // convert from int32_t to int
    // Compress image
    if (cv::imencode("." + format, cv_ptr->image, compressed.data, opts))
    {
      // Optional postprocessing of CV-compressed data
      switch (formatStruct->format)
      {
        case CVTransportCompressionFormat::PPM:
        case CVTransportCompressionFormat::PGM:
        case CVTransportCompressionFormat::PBM:
        case CVTransportCompressionFormat::PNM:
        case CVTransportCompressionFormat::PAM:
        {
          // OpenCV adds a lengthy comment to the start of PNM files. Let's remove it to save some bytes.
          if (compressed.data.size() > 4 && compressed.data[0] == 'P' && compressed.data[3] == '#')
          {
            const auto hashPos = std::next(compressed.data.begin(), 3);
            const auto nlPos = std::find(hashPos, compressed.data.end(), '\n');
            if (nlPos != compressed.data.end())
            {
              compressed.data.erase(hashPos, std::next(nlPos));
            }
          }
          break;
        }

        default:
          break;
      }
      return compressed;
    }
    else
    {
      // The bool return value of imencode is undocumented and it seems it cannot happen, but for completeness...
      return cras::make_unexpected(cras::format(
        "Unknown OpenCV error occurred while encoding %ix%i %s image as %s.",
        raw.width, raw.height, raw.encoding.c_str(), format.c_str()));
    }
  }
  catch (cv_bridge::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "cv_bridge error occurred while encoding %ix%i %s image as %s: %s.",
      raw.width, raw.height, raw.encoding.c_str(), format.c_str(), e.what()));
  }
  catch (cv::Exception& e)
  {
    return cras::make_unexpected(cras::format(
      "OpenCV error occurred while encoding %ix%i %s image as %s: %s (%s).",
      raw.width, raw.height, raw.encoding.c_str(), format.c_str(), cvErrorStr(e.code), e.err.c_str()));
  }
}

CVCodec::EncodeResult CVCodec::encode(
  const sensor_msgs::Image& raw, const CVPublisherConfig& config) const
{
  return this->encode(raw, config.format, configToCvOptions(config));
}

image_transport_codecs::ImageTransportCodec::EncodeResult CVCodec::encode(
  const sensor_msgs::Image& raw, const dynamic_reconfigure::Config& config) const
{
  auto codecConfig = CVPublisherConfig::__getDefault__();
  if (!codecConfig.__fromMessage__(*const_cast<dynamic_reconfigure::Config*>(&config)))
    return cras::make_unexpected("Invalid config passed to cv transport encoder.");

  const auto compressedImage = this->encode(raw, codecConfig);
  if (!compressedImage)
    return cras::make_unexpected(compressedImage.error());

  cras::ShapeShifter compressed;
  cras::msgToShapeShifter(compressedImage.value(), compressed);
  return compressed;
}

image_transport_codecs::ImageTransportCodec::DecodeResult CVCodec::decode(
  const topic_tools::ShapeShifter& compressed, const dynamic_reconfigure::Config& config) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }

  return this->decode(*compressedImage);
}

image_transport_codecs::ImageTransportCodec::GetCompressedContentResult CVCodec::getCompressedImageContent(
  const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const
{
  sensor_msgs::CompressedImageConstPtr compressedImage;
  try
  {
    compressedImage = compressed.instantiate<sensor_msgs::CompressedImage>();
  }
  catch (const ros::Exception& e)
  {
    return cras::make_unexpected(cras::format("Invalid shapeshifter passed to compressed decoder: %s.", e.what()));
  }
  return this->getCompressedImageContent(*compressedImage, matchFormat);
}

image_transport_codecs::ImageTransportCodec::GetCompressedContentResult CVCodec::getCompressedImageContent(
  const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const
{
  const auto formatStruct = parseCompressedTransportFormat(compressed.format);

  if (!cras::startsWith(formatStruct->formatString, cras::toLower(matchFormat)))
    return cras::nullopt;

  return image_transport_codecs::CompressedImageContent{formatStruct->formatString, compressed.data};
}

cras::expected<void, std::string> CVCodec::validateImageFormat(const CVTransportFormat& format) const
{
  const auto f = format.formatString.c_str();

#if CV_VERSION_MAJOR >= 4
  if (!cv::haveImageWriter("." + format.formatString))
#else
  if (!cvHaveImageWriter((std::string(".") + format.formatString).c_str()))
#endif
    return cras::make_unexpected(cras::format("OpenCV cannot encode images with format %s.", f));

  char dataType = 'U';
  if (cras::contains(format.rawEncoding, "SC"))
    dataType = 'S';
  else if (cras::contains(format.rawEncoding, "FC"))
    dataType = 'F';

  switch (format.format)
  {
    case CVTransportCompressionFormat::EXR:
    {
      if (format.bitDepth != 32)
        return cras::make_unexpected(cras::format("Format %s can only handle 32-bit images.", f));
      if (dataType != 'F')
        return cras::make_unexpected(cras::format("Format %s can only handle floating point images.", f));
      break;
    }
    case CVTransportCompressionFormat::HDR:
    case CVTransportCompressionFormat::PIC:
    {
      if (dataType != 'F')
        return cras::make_unexpected(cras::format("Format %s can only handle floating point images.", f));
      break;
    }
    case CVTransportCompressionFormat::JP2:
    case CVTransportCompressionFormat::PPM:
    case CVTransportCompressionFormat::PNM:
    case CVTransportCompressionFormat::PXM:
    {
      if (format.numChannels != 1 && format.numChannels != 3)
        return cras::make_unexpected(cras::format("Format %s can only handle 1/3 channel images.", f));
      if (format.bitDepth != 8 && format.bitDepth != 16)
        return cras::make_unexpected(cras::format("Format %s can only handle 8- or 16-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    case CVTransportCompressionFormat::PNG:
    {
      if (format.numChannels != 1 && format.numChannels != 3 && format.numChannels != 4)
        return cras::make_unexpected(cras::format("Format %s can only handle 1/3/4 channel images.", f));
      if (format.bitDepth != 8 && format.bitDepth != 16)
        return cras::make_unexpected(cras::format("Format %s can only handle 8- or 16-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    case CVTransportCompressionFormat::JXL:
    {
      if (format.numChannels != 1 && format.numChannels != 3 && format.numChannels != 4)
        return cras::make_unexpected(cras::format("Format %s can only handle 1/3/4 channel images.", f));
      if ((dataType == 'U' && (format.bitDepth != 8 && format.bitDepth != 16)) ||
          (dataType == 'F' && format.bitDepth != 32) ||
          (dataType == 'S')
      )
        return cras::make_unexpected(cras::format("Format %s can only handle 8UC, 16UC and 32FC images.", f));
      break;
    }
    case CVTransportCompressionFormat::PAM:
    {
      if (format.bitDepth != 8 && format.bitDepth != 16)
        return cras::make_unexpected(cras::format("Format %s can only handle 8- or 16-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    case CVTransportCompressionFormat::PGM:
    {
      if (format.numChannels != 1)
        return cras::make_unexpected(cras::format("Format %s can only handle 1 channel images.", f));
      if (format.bitDepth != 8 && format.bitDepth != 16)
        return cras::make_unexpected(cras::format("Format %s can only handle 8- or 16-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    case CVTransportCompressionFormat::PBM:
    {
      if (format.numChannels != 1)
        return cras::make_unexpected(cras::format("Format %s can only handle 1 channel images.", f));
      if (format.bitDepth != 8)
        return cras::make_unexpected(cras::format("Format %s can only handle 8-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    case CVTransportCompressionFormat::TIF:
    {
      if ((dataType == 'U' && (format.bitDepth != 8 && format.bitDepth != 16)) ||
          (dataType == 'S' && (format.bitDepth != 8 && format.bitDepth != 16 && format.bitDepth != 32)) ||
          (dataType == 'F' && (format.bitDepth != 32 && format.bitDepth != 64))
      )
        return cras::make_unexpected(
          cras::format("Format %s can only handle 8UC, 8SC, 16UC, 16SC, 32SC, 32FC and 64FC images.", f));
      break;
    }
    case CVTransportCompressionFormat::GIF:
    {
      if (format.numChannels != 3 && format.numChannels != 4)
        return cras::make_unexpected(cras::format("Format %s can only handle 3/4 channel images.", f));
      if (format.bitDepth != 8)
        return cras::make_unexpected(cras::format("Format %s can only handle 8-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
    default:
    {
      if (format.numChannels != 1 && format.numChannels != 3)
        return cras::make_unexpected(cras::format("Format %s can only handle 1/3 channel images.", f));
      if (format.bitDepth != 8)
        return cras::make_unexpected(cras::format("Format %s can only handle 8-bit images.", f));
      if (dataType != 'U')
        return cras::make_unexpected(cras::format("Format %s can only handle unsigned integer images.", f));
      break;
    }
  }
  return {};
}

thread_local auto globalLogger = std::make_shared<cras::MemoryLogHelper>();
thread_local CVCodec cv_codec_instance(globalLogger);
thread_local std::vector<int32_t> globalCvOpts;
}

std::vector<int32_t> cv_image_transport::configToCvOptions(const CVPublisherConfig& config)
{
  std::vector<int32_t> options;

  const auto def = CVPublisherConfig::__getDefault__();

  const auto append = [&options](const int key, const int value)
  {
    options.reserve(options.size() + 2);
    options.push_back(key);
    options.push_back(value);
  };

  if (config.format == CVPublisher_format_jpeg)
  {
    append(cv::IMWRITE_JPEG_QUALITY, config.jpeg_quality);
    if (config.jpeg_progressive != def.jpeg_progressive)
      append(cv::IMWRITE_JPEG_PROGRESSIVE, config.jpeg_progressive);
    if (config.jpeg_optimize != def.jpeg_optimize)
      append(cv::IMWRITE_JPEG_OPTIMIZE, config.jpeg_optimize);
    if (config.jpeg_rst_interval != def.jpeg_rst_interval)
      append(cv::IMWRITE_JPEG_RST_INTERVAL, config.jpeg_rst_interval);
    if (config.jpeg_luma_quality != def.jpeg_luma_quality)
      append(cv::IMWRITE_JPEG_LUMA_QUALITY, config.jpeg_luma_quality);
    if (config.jpeg_chroma_quality != def.jpeg_chroma_quality)
      append(cv::IMWRITE_JPEG_CHROMA_QUALITY, config.jpeg_chroma_quality);
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7) || \
  (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 4 && CV_VERSION_REVISION >= 19)
    if (config.jpeg_sampling_factor != def.jpeg_sampling_factor)
      append(cv::IMWRITE_JPEG_SAMPLING_FACTOR, config.jpeg_sampling_factor);
#endif
  }
  else if (config.format == CVPublisher_format_png)
  {
    append(cv::IMWRITE_PNG_COMPRESSION, config.png_level);
    if (config.png_strategy != def.png_strategy)
      append(cv::IMWRITE_PNG_STRATEGY, config.png_strategy);
    if (config.png_bilevel != def.png_bilevel)
      append(cv::IMWRITE_PNG_BILEVEL, config.png_bilevel);
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 12)
    if (config.png_filter != def.png_filter)
      append(cv::IMWRITE_PNG_FILTER, config.png_filter);
#endif
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 13)
    if (config.png_zlib_buffer_size != def.png_zlib_buffer_size)
      append(cv::IMWRITE_PNG_ZLIBBUFFER_SIZE, config.png_zlib_buffer_size);
#endif
  }
  else if (config.format == CVPublisher_format_tif)
  {
#if CV_VERSION_MAJOR >= 4 || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 4 && CV_VERSION_REVISION >= 2)
    if (config.tiff_resunit != def.tiff_resunit)
      append(cv::IMWRITE_TIFF_RESUNIT, config.tiff_resunit);
    if (config.tiff_xdpi != def.tiff_xdpi)
      append(cv::IMWRITE_TIFF_XDPI, config.tiff_xdpi);
    if (config.tiff_ydpi != def.tiff_ydpi)
      append(cv::IMWRITE_TIFF_YDPI, config.tiff_ydpi);
#endif
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 1)
    if (config.tiff_compression != def.tiff_compression)
      append(cv::IMWRITE_TIFF_COMPRESSION, config.tiff_compression);
#endif
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 10)
    if (config.tiff_rows_per_strip != def.tiff_rows_per_strip)
      append(cv::IMWRITE_TIFF_ROWSPERSTRIP, config.tiff_rows_per_strip);
    if (config.tiff_predictor != def.tiff_predictor)
      append(cv::IMWRITE_TIFF_PREDICTOR, config.tiff_predictor);
#endif
  }
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 1)
  else if (config.format == CVPublisher_format_jp2)
  {
    if (config.jpeg2000_compression_x1000 != def.jpeg2000_compression_x1000)
      append(cv::IMWRITE_JPEG2000_COMPRESSION_X1000, config.jpeg2000_compression_x1000);
  }
#endif
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 11)
  else if (config.format == CVPublisher_format_jxl)
  {
    if (config.jpegxl_quality != def.jpegxl_quality)
      append(cv::IMWRITE_JPEGXL_QUALITY, config.jpegxl_quality);
    if (config.jpegxl_effort != def.jpegxl_effort)
      append(cv::IMWRITE_JPEGXL_EFFORT, config.jpegxl_effort);
    if (config.jpegxl_distance != def.jpegxl_distance)
      append(cv::IMWRITE_JPEGXL_DISTANCE, config.jpegxl_distance);
    if (config.jpegxl_decoding_speed != def.jpegxl_decoding_speed)
      append(cv::IMWRITE_JPEGXL_DECODING_SPEED, config.jpegxl_decoding_speed);
  }
#endif
  else if (config.format == CVPublisher_format_pbm || config.format == CVPublisher_format_pgm ||
    config.format == CVPublisher_format_ppm || config.format == CVPublisher_format_pnm)
  {
    append(cv::IMWRITE_PXM_BINARY, config.pxm_binary);
  }
  else if (config.format == CVPublisher_format_pam)
  {
    append(cv::IMWRITE_PAM_TUPLETYPE, config.pam_tupletype);
  }
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 11)
  else if (config.format == CVPublisher_format_gif)
  {
    if (config.gif_quality != def.gif_quality)
      append(cv::IMWRITE_GIF_QUALITY, config.gif_quality);
    if (config.gif_dither != def.gif_dither)
      append(cv::IMWRITE_GIF_DITHER, config.gif_dither);
    if (config.gif_transparency != def.gif_transparency)
      append(cv::IMWRITE_GIF_TRANSPARENCY, config.gif_transparency);
    if (config.gif_colortable != def.gif_colortable)
      append(cv::IMWRITE_GIF_COLORTABLE, config.gif_colortable);
  }
#endif
  else if (config.format == CVPublisher_format_webp)
  {
    append(cv::IMWRITE_WEBP_QUALITY, config.webp_quality);
  }
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 8)
  else if (config.format == CVPublisher_format_avif)
  {
    if (config.avif_quality != def.avif_quality)
      append(cv::IMWRITE_AVIF_QUALITY, config.avif_quality);
    if (config.avif_depth != def.avif_depth)
      append(cv::IMWRITE_AVIF_DEPTH, config.avif_depth);
    if (config.avif_speed != def.avif_speed)
      append(cv::IMWRITE_AVIF_SPEED, config.avif_speed);
  }
#endif
#if CV_VERSION_MAJOR >= 4 || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR >= 4)
  else if (config.format == CVPublisher_format_exr)
  {
    if (config.exr_type != def.exr_type)
      append(cv::IMWRITE_EXR_TYPE, config.exr_type);
#if CV_VERSION_MAJOR >= 5 || \
    (CV_VERSION_MAJOR == 4 && (CV_VERSION_MINOR >= 6 || (CV_VERSION_MINOR == 5 && CV_VERSION_REVISION >= 2)))
    if (config.exr_compression != def.exr_compression)
      append(cv::IMWRITE_EXR_COMPRESSION, config.exr_compression);
#endif
#if CV_VERSION_MAJOR >= 5 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
    if (config.exr_dwa_compression_level != def.exr_dwa_compression_level)
      append(cv::IMWRITE_EXR_DWA_COMPRESSION_LEVEL, config.exr_dwa_compression_level);
#endif
  }
#endif
#if CV_VERSION_MAJOR >= 5 || \
    (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7) || \
    (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 4 && CV_VERSION_REVISION >= 19)
  else if (config.format == CVPublisher_format_hdr || config.format == CVPublisher_format_pic)
  {
    if (config.hdr_compression != def.hdr_compression)
      append(cv::IMWRITE_HDR_COMPRESSION, config.hdr_compression);
  }
#endif

  return options;
}

cv_image_transport::CVPublisherConfig convertCVConfig(const cv_image_transport_CVPublisherConfig& config)
{
  cv_image_transport::CVPublisherConfig result;
  result.format = config.format;
  result.enable = config.enable;
  result.jpeg_quality = config.jpeg_quality;
  result.jpeg_progressive = config.jpeg_progressive;
  result.jpeg_optimize = config.jpeg_optimize;
  result.jpeg_rst_interval = config.jpeg_rst_interval;
  result.jpeg_luma_quality = config.jpeg_luma_quality;
  result.jpeg_chroma_quality = config.jpeg_chroma_quality;
  result.jpeg_sampling_factor = config.jpeg_sampling_factor;
  result.png_level = config.png_level;
  result.png_strategy = config.png_strategy;
  result.png_bilevel = config.png_bilevel;
  result.png_filter = config.png_filter;
  result.png_zlib_buffer_size = config.png_zlib_buffer_size;
  result.pxm_binary = config.pxm_binary;
  result.exr_type = config.exr_type;
  result.exr_compression = config.exr_compression;
  result.exr_dwa_compression_level = config.exr_dwa_compression_level;
  result.webp_quality = config.webp_quality;
  result.hdr_compression = config.hdr_compression;
  result.pam_tupletype = config.pam_tupletype;
  result.tiff_resunit = config.tiff_resunit;
  result.tiff_xdpi = config.tiff_xdpi;
  result.tiff_ydpi = config.tiff_ydpi;
  result.tiff_compression = config.tiff_compression;
  result.tiff_rows_per_strip = config.tiff_rows_per_strip;
  result.tiff_predictor = config.tiff_predictor;
  result.jpeg2000_compression_x1000 = config.jpeg2000_compression_x1000;
  result.avif_quality = config.avif_quality;
  result.avif_depth = config.avif_depth;
  result.avif_speed = config.avif_speed;
  result.jpegxl_quality = config.jpegxl_quality;
  result.jpegxl_effort = config.jpegxl_effort;
  result.jpegxl_distance = config.jpegxl_distance;
  result.jpegxl_decoding_speed = config.jpegxl_decoding_speed;
  result.gif_quality = config.gif_quality;
  result.gif_dither = config.gif_dither;
  result.gif_transparency = config.gif_transparency;
  result.gif_colortable = config.gif_colortable;
  return result;
}

void cvConfigToCVOpts(const cv_image_transport_CVPublisherConfig* config, cras::allocator_t cvOptsAllocator)
{
  const auto cvOpts = cv_image_transport::configToCvOptions(convertCVConfig(*config));

  auto result = static_cast<int32_t*>(cvOptsAllocator(cvOpts.size()));
  std::memcpy(result, cvOpts.data(), sizeof(int32_t) * cvOpts.size());
}

bool cvCodecEncode(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  const size_t rawDataLength,
  const uint8_t rawData[],
  const cv_image_transport_CVPublisherConfig* config,
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator)
{
  using cv_image_transport::globalCvOpts;
  const auto alloc = [](const size_t size)
  {
    globalCvOpts.resize(size);
    return static_cast<void*>(globalCvOpts.data());
  };
  cvConfigToCVOpts(config, alloc);

  return cvCodecEncodeWithCVOpts(rawHeight, rawWidth, rawEncoding, rawIsBigEndian, rawStep, rawDataLength, rawData,
    config->format, globalCvOpts.size(), globalCvOpts.data(),
    compressedFormatAllocator, compressedDataAllocator, errorStringAllocator, logMessagesAllocator);
}

bool cvCodecEncodeWithCVOpts(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  const size_t rawDataLength,
  const uint8_t rawData[],
  const char* format,
  const size_t cvOptsLength,
  const int32_t cvOpts[],
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator)
{
  sensor_msgs::Image raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  raw.encoding = rawEncoding;
  raw.is_bigendian = rawIsBigEndian;
  raw.step = rawStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);

  cv_image_transport::globalLogger->clear();

  const std::vector<int32_t> cvOptions(cvOpts, cvOpts + cvOptsLength);

  const auto compressed = cv_image_transport::cv_codec_instance.encode(raw, format, cvOptions);

  for (const auto& msg : cv_image_transport::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  cv_image_transport::globalLogger->clear();

  if (!compressed)
  {
    cras::outputString(errorStringAllocator, compressed.error());
    return false;
  }

  cras::outputString(compressedFormatAllocator, compressed->format);
  cras::outputByteBuffer(compressedDataAllocator, compressed->data);

  return true;
}

bool cvCodecDecode(
  const char* compressedFormat,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::Image::_height_type& rawHeight,
  sensor_msgs::Image::_width_type& rawWidth,
  cras::allocator_t rawEncodingAllocator,
  sensor_msgs::Image::_is_bigendian_type& rawIsBigEndian,
  sensor_msgs::Image::_step_type& rawStep,
  cras::allocator_t rawDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
)
{
  sensor_msgs::CompressedImage compressed;
  compressed.format = compressedFormat;
  compressed.data.resize(compressedDataLength);
  memcpy(compressed.data.data(), compressedData, compressedDataLength);

  cv_image_transport::globalLogger->clear();

  const auto raw = cv_image_transport::cv_codec_instance.decode(compressed);

  for (const auto& msg : cv_image_transport::globalLogger->getMessages())
    cras::outputRosMessage(logMessagesAllocator, msg);
  cv_image_transport::globalLogger->clear();

  if (!raw)
  {
    cras::outputString(errorStringAllocator, raw.error());
    return false;
  }

  rawHeight = raw->height;
  rawWidth = raw->width;
  rawIsBigEndian = raw->is_bigendian;
  rawStep = raw->step;
  cras::outputString(rawEncodingAllocator, raw->encoding);
  cras::outputByteBuffer(rawDataAllocator, raw->data);

  return true;
}
