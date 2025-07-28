#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Image transport codec working with CV format.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cv_image_transport/CVPublisherConfig.h>
#include <cv_image_transport/parse_compressed_format.h>
#include <image_transport_codecs/image_transport_codec.h>

namespace cv_image_transport
{

struct CVCodecPrivate;

/**
 * \brief Image transport codec for OpenCV formats.
 *
 * This codec exposes the functionality of `cv_image_transport` so that it can be used directly without the
 * need to go through running a node and publishing on a topic. E.g.
 *
 * ```
 * sensor_msgs::Image raw = ...;
 * cv_image_transport::CVCodec codec;
 * auto result = codec.encode(raw);
 * if (!result)
 * {
 *   ROS_ERROR_STREAM("Compression failed: " << result.error());
 *   return false;
 * }
 * sensor_msgs::CompressedImage compressed = result.value();
 * ```
 */
class CVCodec : public image_transport_codecs::ImageTransportCodec
{
public:
  //! \brief Result of image encoding. Either a `sensor_msgs::CompressedImage` message, or error message.
  typedef cras::expected<sensor_msgs::CompressedImage, std::string> EncodeResult;

  /**
   * \brief Create an instance of the codec.
   * \param[in] logHelper The logger to use for error messages not directly related to the currently processed image.
   */
  explicit CVCodec(const cras::LogHelperPtr& logHelper = std::make_shared<cras::NodeLogHelper>());

  ~CVCodec() override;

  /**
   * \brief Encode the given raw image using the given publisher config.
   * \param[in] raw The raw image to encode.
   * \param[in] config The encoder configuration.
   * \sa Corresponding C API function: `cvCodecEncode()`.
   */
  EncodeResult encode(const sensor_msgs::Image& raw, const CVPublisherConfig& config) const;

  /**
   * \brief Encode the given raw image using the given publisher config.
   * \param[in] raw The raw image to encode.
   * \param[in] format The image format.
   * \param[in] cvOpts OpenCV imencode options.
   * \sa Corresponding C API function: `cvCodecEncode()`.
   */
  EncodeResult encode(const sensor_msgs::Image& raw,
    const std::string& format, const std::vector<int32_t>& cvOpts) const;

  /**
   * \brief Decode the given compressed image.
   * \param[in] compressed The image to decode.
   * \return The decoded raw image, or an error.
   * \sa Corresponding C API function: `cvCodecDecode()`.
   */
  ImageTransportCodec::DecodeResult decode(const sensor_msgs::CompressedImage& compressed) const;

  std::string getTransportName() const override;

  ImageTransportCodec::EncodeResult encode(const sensor_msgs::Image& raw,
                                           const dynamic_reconfigure::Config& config) const override;

  ImageTransportCodec::DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                                           const dynamic_reconfigure::Config& config) const override;

  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const topic_tools::ShapeShifter& compressed, const std::string& matchFormat) const override;

  /**
   * \brief This function returns the bytes of the actual CV image.
   * \param[in] compressed The compressed image.
   * \param[in] matchFormat If nonempty, the image data is only returned if their `format` field would match the given
   *                        one. The matching should be case-insensitive.
   * \return The contained image bytes.
   */
  ImageTransportCodec::GetCompressedContentResult getCompressedImageContent(
    const sensor_msgs::CompressedImage& compressed, const std::string& matchFormat) const;

  /**
   * Validate whether the given image format is valid for being encoded.
   * \param[in] format The parsed format.
   * \return Nothing is the format is good. Error string in case the format is invalid.
   */
  cras::expected<void, std::string> validateImageFormat(const CVTransportFormat& format) const;

private:
  std::unique_ptr<CVCodecPrivate> data;  //!< \brief Private implementation data
};

std::vector<int32_t> configToCvOptions(const CVPublisherConfig& config);

}

// ////////
// C API //
// ////////

/**
 * \brief This struct reflects the data members of cv_image_transport::CVPublisherConfig .
 */
struct cv_image_transport_CVPublisherConfig
{
  const char* format;
  bool enable;
  int jpeg_quality;
  bool jpeg_progressive;
  bool jpeg_optimize;
  int jpeg_rst_interval;
  int jpeg_luma_quality;
  int jpeg_chroma_quality;
  int jpeg_sampling_factor;
  int png_level;
  int png_strategy;
  bool png_bilevel;
  int png_filter;
  int png_zlib_buffer_size;
  bool pxm_binary;
  int exr_type;
  int exr_compression;
  int exr_dwa_compression_level;
  int webp_quality;
  int hdr_compression;
  int pam_tupletype;
  int tiff_resunit;
  int tiff_xdpi;
  int tiff_ydpi;
  int tiff_compression;
  int tiff_rows_per_strip;
  int tiff_predictor;
  int jpeg2000_compression_x1000;
  int avif_quality;
  int avif_depth;
  int avif_speed;
  int jpegxl_quality;
  int jpegxl_effort;
  int jpegxl_distance;
  int jpegxl_decoding_speed;
  int gif_quality;
  int gif_dither;
  int gif_transparency;
  int gif_colortable;
};

/**
 * \brief Convert the given encoder config to CV imencode options.
 *
 * \param[in] config The encoder config.
 * \param[in,out] cvOptsAllocator Allocator for the parsed CV options. It should allocate 4-byte int array.
 */
extern "C" void cvConfigToCVOpts(
  const cv_image_transport_CVPublisherConfig* config,
  cras::allocator_t cvOptsAllocator
);

/**
 * \brief Encode the given raw image using `cv` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] rawHeight Raw image height, that is, number of rows.
 * \param[in] rawWidth Raw image width, that is, number of columns.
 * \param[in] rawEncoding Raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[in] rawIsBigEndian Is raw image bigendian?
 * \param[in] rawStep Raw image full row length in bytes.
 * \param[in] rawDataLength Length of raw image data in bytes, should be `step * rows`.
 * \param[in] rawData The raw image bytes.
 * \param[in] config The encoder configuration.
 * \param[in,out] compressedFormatAllocator Allocator for the `format` field of the compressed image.
 * \param[in,out] compressedDataAllocator Allocator for the byte data of the compressed image.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, `compressedFormatAllocator` and `compressedDataAllocator`
 *         allocate their buffers and write the output to them. If not, `errorStringAllocator` allocates its buffer
 *         and stores the error string in it.
 * \sa Corresponding C++ API function: `cv_image_transport::CvCodec::encode()`.
 */
extern "C" bool cvCodecEncode(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  const cv_image_transport_CVPublisherConfig* config,
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);

/**
 * \brief Encode the given raw image using `cv` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] rawHeight Raw image height, that is, number of rows.
 * \param[in] rawWidth Raw image width, that is, number of columns.
 * \param[in] rawEncoding Raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[in] rawIsBigEndian Is raw image bigendian?
 * \param[in] rawStep Raw image full row length in bytes.
 * \param[in] rawDataLength Length of raw image data in bytes, should be `step * rows`.
 * \param[in] rawData The raw image bytes.
 * \param[in] format The output format of the image (i.e. image format file extension).
 * \param[in] cvOptsLength Length of the OpenCV options array.
 * \param[in] cvOpts OpenCV imencode options array.
 * \param[in,out] compressedFormatAllocator Allocator for the `format` field of the compressed image.
 * \param[in,out] compressedDataAllocator Allocator for the byte data of the compressed image.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, `compressedFormatAllocator` and `compressedDataAllocator`
 *         allocate their buffers and write the output to them. If not, `errorStringAllocator` allocates its buffer
 *         and stores the error string in it.
 * \sa Corresponding C++ API function: `cv_image_transport::CvCodec::encode()`.
 */
extern "C" bool cvCodecEncodeWithCVOpts(
  sensor_msgs::Image::_height_type rawHeight,
  sensor_msgs::Image::_width_type rawWidth,
  const char* rawEncoding,
  sensor_msgs::Image::_is_bigendian_type rawIsBigEndian,
  sensor_msgs::Image::_step_type rawStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  const char* format,
  size_t cvOptsLength,
  const int32_t cvOpts[],
  cras::allocator_t compressedFormatAllocator,
  cras::allocator_t compressedDataAllocator,
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);

/**
 * \brief Decode the given compressed image using `cv` codec with the given config.
 *
 * This is a *C API* to allow interfacing this library from other programming languages. Do not use it in C++.
 *
 * \param[in] compressedFormat The `format` field of the compressed image.
 * \param[in] compressedDataLength Length of the compressed image data in bytes.
 * \param[in] compressedData Bytes of the compressed image.
 * \param[out] rawHeight Raw image height, that is, number of rows.
 * \param[out] rawWidth Raw image width, that is, number of columns.
 * \param[in,out] rawEncodingAllocator Allocator for raw image encoding of pixels -- channel meaning, ordering, size.
 * \param[out] rawIsBigEndian Is raw image bigendian?
 * \param[out] rawStep Raw image full row length in bytes.
 * \param[in,out] rawDataAllocator Allocator for raw image bytes.
 * \param[in,out] errorStringAllocator Allocator for error string in case the encoding fails.
 * \param[in,out] logMessagesAllocator Allocator for log messages to be passed to the calling code. Each allocated
 *                                     message should be properly reported by the native logging mechanism after this
 *                                     call finishes. The messages are serialized `rosgraph_msgs::Log` messages.
 * \return Whether the encoding has succeeded. If yes, output parameters are set, `rawEncodingAllocator` and
 *         `rawDataAllocator` allocate their buffers and write the output to them. If not, `errorStringAllocator`
 *         allocates its buffer and stores the error string in it.
 * \sa Corresponding C++ API function: `cv_image_transport::CvCodec::decode()`.
 */
extern "C" bool cvCodecDecode(
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
);
