/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/image_encodings.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cv_image_transport/CVPublisherConfig.h>
#include <cv_image_transport/parse_compressed_format.h>

namespace cv_image_transport
{

std::unordered_map<std::string, CVTransportCompressionFormat> compressedFormatTypes =
{
  {CVPublisher_format_jpeg, CVTransportCompressionFormat::JPEG},
  {CVPublisher_format_png, CVTransportCompressionFormat::PNG},
  {CVPublisher_format_tif, CVTransportCompressionFormat::TIF},
  {CVPublisher_format_bmp, CVTransportCompressionFormat::BMP},
  {CVPublisher_format_dib, CVTransportCompressionFormat::DIB},
  {CVPublisher_format_jp2, CVTransportCompressionFormat::JP2},
  {CVPublisher_format_jxl, CVTransportCompressionFormat::JXL},
  {CVPublisher_format_pbm, CVTransportCompressionFormat::PBM},
  {CVPublisher_format_pgm, CVTransportCompressionFormat::PGM},
  {CVPublisher_format_ppm, CVTransportCompressionFormat::PPM},
  {CVPublisher_format_pxm, CVTransportCompressionFormat::PXM},
  {CVPublisher_format_pnm, CVTransportCompressionFormat::PNM},
  {CVPublisher_format_pam, CVTransportCompressionFormat::PAM},
  {CVPublisher_format_sr, CVTransportCompressionFormat::SR},
  {CVPublisher_format_ras, CVTransportCompressionFormat::RAS},
  {CVPublisher_format_gif, CVTransportCompressionFormat::GIF},
  {CVPublisher_format_webp, CVTransportCompressionFormat::WEBP},
  {CVPublisher_format_avif, CVTransportCompressionFormat::AVIF},
  {CVPublisher_format_exr, CVTransportCompressionFormat::EXR},
  {CVPublisher_format_hdr, CVTransportCompressionFormat::HDR},
  {CVPublisher_format_pic, CVTransportCompressionFormat::PIC},
  {"jpg", CVTransportCompressionFormat::JPEG},
  {"tiff", CVTransportCompressionFormat::JPEG},
};

std::unordered_map<CVTransportCompressionFormat, std::string> compressedFormatNames =
{
  {CVTransportCompressionFormat::JPEG, CVPublisher_format_jpeg},
  {CVTransportCompressionFormat::PNG, CVPublisher_format_png},
  {CVTransportCompressionFormat::TIF, CVPublisher_format_tif},
  {CVTransportCompressionFormat::BMP, CVPublisher_format_bmp},
  {CVTransportCompressionFormat::DIB, CVPublisher_format_dib},
  {CVTransportCompressionFormat::JP2, CVPublisher_format_jp2},
  {CVTransportCompressionFormat::JXL, CVPublisher_format_jxl},
  {CVTransportCompressionFormat::PBM, CVPublisher_format_pbm},
  {CVTransportCompressionFormat::PGM, CVPublisher_format_pgm},
  {CVTransportCompressionFormat::PPM, CVPublisher_format_ppm},
  {CVTransportCompressionFormat::PXM, CVPublisher_format_pxm},
  {CVTransportCompressionFormat::PNM, CVPublisher_format_pnm},
  {CVTransportCompressionFormat::PAM, CVPublisher_format_pam},
  {CVTransportCompressionFormat::SR, CVPublisher_format_sr},
  {CVTransportCompressionFormat::RAS, CVPublisher_format_ras},
  {CVTransportCompressionFormat::GIF, CVPublisher_format_gif},
  {CVTransportCompressionFormat::WEBP, CVPublisher_format_webp},
  {CVTransportCompressionFormat::AVIF, CVPublisher_format_avif},
  {CVTransportCompressionFormat::EXR, CVPublisher_format_exr},
  {CVTransportCompressionFormat::HDR, CVPublisher_format_hdr},
  {CVTransportCompressionFormat::PIC, CVPublisher_format_pic},
};

cras::expected<CVTransportFormat, std::string> parseCompressedTransportFormat(const std::string& format)
{
  const auto parts = cras::split(format, ";", 1);
  if (parts.size() == 1)
    return cras::make_unexpected("cv transport format '" + format + "' is invalid.");

  // The general format is:
  // RAW_PIXFMT; CODEC compressed [COMPRESSED_PIXFMT]

  const auto rawEncoding = cras::strip(parts[0]);

  const auto parts2 = cras::split(cras::strip(parts[1]), " ");
  if (parts2.size() < 2 || cras::strip(parts2[1]) != "compressed")
    return cras::make_unexpected("compressed transport format '" + format + "' is invalid.");

  const auto f = cras::strip(parts2[0]);
  if (compressedFormatTypes.find(f) == compressedFormatTypes.end())
    return cras::make_unexpected("unknown compression format '" + f + "' for cv transport.");

  const auto formatType = compressedFormatTypes[f];

  auto result = extractCompressedTransportFormat(rawEncoding, formatType);

  if (parts2.size() > 2)
  {
    const auto enc = cras::strip(parts2[2]);
    if (!enc.empty())
      result.compressedEncoding = enc;
  }

  return result;
}

std::string makeCompressedTransportFormat(const CVTransportFormat& format)
{
  if (format.formatString.empty() || format.bitDepth <= 0 || format.numChannels <= 0 || format.rawEncoding.empty())
    return "";

  std::string compressedEncoding{};
  if (format.isColor)
  {
    using CF = CVTransportCompressionFormat;
    if (format.format == CF::JPEG || format.format == CF::PNG)
      compressedEncoding = format.compressedEncoding;
    else if (format.compressedEncoding != "bgr8" && format.compressedEncoding != "bgra8" &&
        format.rawEncoding != format.compressedEncoding)
      compressedEncoding = format.compressedEncoding;
  }

  return cras::format("%s; %s compressed %s", format.rawEncoding.c_str(), format.formatString.c_str(),
                      compressedEncoding.c_str());
}

CVTransportFormat extractCompressedTransportFormat(
  const std::string& imageEncoding, const CVTransportCompressionFormat& compressionFormat)
{
  CVTransportFormat result;
  result.format = compressionFormat;
  result.formatString = compressedFormatNames[result.format];
  result.rawEncoding = result.compressedEncoding = imageEncoding;
  result.isColor = sensor_msgs::image_encodings::isColor(result.rawEncoding);
  result.isMono = sensor_msgs::image_encodings::isMono(result.rawEncoding);
  result.isBayer = sensor_msgs::image_encodings::isBayer(result.rawEncoding);
  result.hasAlpha = sensor_msgs::image_encodings::hasAlpha(result.rawEncoding);
  try
  {
    result.bitDepth = sensor_msgs::image_encodings::bitDepth(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.bitDepth = 8;
  }

  try
  {
    result.numChannels = sensor_msgs::image_encodings::numChannels(result.rawEncoding);
  }
  catch (const std::runtime_error&)
  {
    result.numChannels = result.isColor ? 3 : 1;
  }

  if (result.isColor)
  {
    result.compressedEncoding =
      result.bitDepth == 8 ? sensor_msgs::image_encodings::BGR8 : sensor_msgs::image_encodings::BGR16;
  }

  return result;
}

CVTransportFormat extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const CVTransportCompressionFormat& compressionFormat)
{
  return extractCompressedTransportFormat(image.encoding, compressionFormat);
}

cras::expected<CVTransportFormat, std::string> extractCompressedTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat)
{
  if (compressedFormatTypes.find(compressionFormat) == compressedFormatTypes.end())
    return cras::make_unexpected("Unknown compressed transport format '" + compressionFormat + "'.");
  const auto& format2 = compressedFormatTypes[compressionFormat];
  return extractCompressedTransportFormat(imageEncoding, format2);
}

cras::expected<CVTransportFormat, std::string> extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat)
{
  return extractCompressedTransportFormat(image.encoding, compressionFormat);
}


bool CVTransportFormat::operator==(const CVTransportFormat& other) const
{
  return
    this->format == other.format &&
    this->formatString == other.formatString &&
    this->rawEncoding == other.rawEncoding &&
    this->compressedEncoding == other.compressedEncoding &&
    this->numChannels == other.numChannels &&
    this->bitDepth == other.bitDepth &&
    this->isColor == other.isColor &&
    this->isMono == other.isMono &&
    this->isBayer == other.isBayer &&
    this->hasAlpha == other.hasAlpha;
}

}

bool parseCVTransportFormat(
  const char* format,
  cras::allocator_t compressionFormatAllocator,
  cras::allocator_t rawEncodingAllocator,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  bool& isMono,
  bool& isBayer,
  bool& hasAlpha,
  cras::allocator_t errorStringAllocator)
{
  const auto parsed = cv_image_transport::parseCompressedTransportFormat(format);
  if (!parsed)
  {
    cras::outputString(errorStringAllocator, parsed.error());
    return false;
  }

  cras::outputString(compressionFormatAllocator, parsed->formatString);
  cras::outputString(rawEncodingAllocator, parsed->rawEncoding);
  cras::outputString(compressedEncodingAllocator, parsed->compressedEncoding);

  numChannels = parsed->numChannels;
  bitDepth = parsed->bitDepth;
  isColor = parsed->isColor;
  isMono = parsed->isMono;
  isBayer = parsed->isBayer;
  hasAlpha = parsed->hasAlpha;

  return true;
}

bool makeCVTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  const char* compressedEncoding,
  const int numChannels,
  const int bitDepth,
  const bool isColor,
  const bool isMono,
  const bool isBayer,
  const bool hasAlpha,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator)
{
  namespace cit = cv_image_transport;
  if (cit::compressedFormatTypes.find(compressionFormat) == cit::compressedFormatTypes.end())
  {
    cras::outputString(errorStringAllocator,
      cras::format("Unknown compressed transport format '%s'.", compressionFormat));
    return false;
  }
  cit::CVTransportFormat format {cit::compressedFormatTypes[compressionFormat], compressionFormat,
    rawEncoding, compressedEncoding, numChannels, bitDepth, isColor, isMono, isBayer, hasAlpha};
  cras::outputString(formatAllocator, cit::makeCompressedTransportFormat(format));
  return true;
}

bool extractCVTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  bool& isMono,
  bool& isBayer,
  bool& hasAlpha,
  cras::allocator_t errorStringAllocator)
{
  const auto format = cv_image_transport::extractCompressedTransportFormat(imageEncoding, compressionFormat);
  if (!format)
  {
    cras::outputString(errorStringAllocator, format.error());
    return false;
  }
  cras::outputString(compressedEncodingAllocator, format->compressedEncoding);
  numChannels = format->numChannels;
  bitDepth = format->bitDepth;
  isColor = format->isColor;
  isMono = format->isMono;
  isBayer = format->isBayer;
  hasAlpha = format->hasAlpha;

  return true;
}
