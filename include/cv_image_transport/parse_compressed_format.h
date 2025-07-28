#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Methods for parsing the values of field `sensor_msgs::CompressedImage::format` for `cv` codec.
 * \author Martin Pecka
 */

#include <string>
#include <utility>

#include <sensor_msgs/Image.h>

#include <cras_cpp_common/c_api.h>
#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>

namespace cv_image_transport
{

/**
 * \brief Compression format of `cv` codec.
 */
enum class CVTransportCompressionFormat : uint8_t
{
  JPEG,  //!< JPEG compression format
  PNG,  //!< PNG compression format
  TIF,  //!< TIF compression format
  BMP,  //!< BMP compression format
  DIB,  //!< DIB compression format
  JP2,  //!< JPEG 2000 compression format
  JXL,  //!< JPEG XL compression format
  PBM,  //!< PBM compression format
  PGM,  //!< PGM compression format
  PPM,  //!< PPM compression format
  PXM,  //!< PXM compression format
  PNM,  //!< PNM compression format
  PAM,  //!< PAM compression format
  SR,  //!< SR compression format
  RAS,  //!< RAS compression format
  GIF,  //!< GIF compression format
  WEBP,  //!< WEBP compression format
  AVIF,  //!< AVIF compression format
  EXR,  //!< EXR compression format
  HDR,  //!< HDR compression format
  PIC,  //!< PIC compression format
  RESERVED1,
  RESERVED2,
  RESERVED3,
  RESERVED4,
  RESERVED5,
  RESERVED6,
  RESERVED7,
  RESERVED8,
  RESERVED9,
};

/**
 * \brief Decoded meaning of field `sensor_msgs::CompressedImage::format` for `cv` transport.
 */
struct CVTransportFormat
{
  CVTransportCompressionFormat format;  //!< \brief The compression format.
  std::string formatString;  //!< \brief Text version of the compression format ("jpeg"/"png" etc.).
  std::string rawEncoding;  //!< \brief Encoding of the raw image (before compression, after decompression).
  std::string compressedEncoding;  //!< \brief Encoding of the compressed image (i.e. `bgr8` for JPEG).
  int numChannels;  //!< \brief Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
  int bitDepth;  //!< \brief Number of bits used for encoding one raw channel value.
  bool isColor;  //!< \brief Whether the image is a color image or not.
  bool isMono;  //!< \brief Whether the image is a mono image or not.
  bool isBayer;  //!< \brief Whether the image is a bayer image or not.
  bool hasAlpha;  //!< \brief Whether the image has an alpha channel or not.

  bool operator==(const CVTransportFormat& other) const;
};

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `cv` transport into
 *        `CVTransportFormat` structure.
 * \param[in] format The `format` field text.
 * \return The parsed structure or error string.
 */
cras::expected<CVTransportFormat, std::string> parseCompressedTransportFormat(const std::string& format);

/**
 * \brief Convert the `CVTransportFormat` structure into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format` of `cv` transport image.
 * \param[in] format The format to convert.
 * \return The string for the `format` field.
 */
std::string makeCompressedTransportFormat(const CVTransportFormat& format);

/**
 * \brief Create the `CVTransportFormat` structure for the given raw image compressed with the given method.
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CVTransportFormat` structure corresponding to the given image and compression method.
 */
CVTransportFormat extractCompressedTransportFormat(
  const std::string& imageEncoding, const CVTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CVTransportFormat` structure for the given raw image compressed with the given method.
 * \param[in] image The raw image.
 * \param[in] compressionFormat The target compression method.
 * \return The `CVTransportFormat` structure corresponding to the given image and compression method.
 */
CVTransportFormat extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const CVTransportCompressionFormat& compressionFormat);

/**
 * \brief Create the `CVTransportFormat` structure for the given raw image compressed with the given method.
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The string version of target compression method (either "jpeg" or "png").
 * \return The `CVTransportFormat` structure corresponding to the given image and compression method, or error
 *         string if `format` is invalid.
 */
cras::expected<CVTransportFormat, std::string> extractCompressedTransportFormat(
  const std::string& imageEncoding, const std::string& compressionFormat);

/**
 * \brief Create the `CVTransportFormat` structure for the given raw image compressed with the given method.
 * \param[in] image The raw image.
 * \param[in] compressionFormat The string version of target compression method (either "jpeg" or "png").
 * \return The `CVTransportFormat` structure corresponding to the given image and compression method, or error
 *         string if `format` is invalid.
 */
cras::expected<CVTransportFormat, std::string> extractCompressedTransportFormat(
  const sensor_msgs::Image& image, const std::string& compressionFormat);

}

/////////////
/// C API ///
/////////////

/**
 * \brief Parse the string from field `sensor_msgs::CompressedImage::format` using `cv` transport into
 *        individual components.
 * \param[in] format The `format` field text.
 * \param[in,out] compressionFormatAllocator Allocator for the compression format ("jpeg", "png" etc.).
 * \param[in,out] rawEncodingAllocator Allocator for encoding of the raw image (before compression, after
 *                                     decompression).
 * \param[in,out] compressedEncodingAllocator Allocator for encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[out] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[out] isColor Whether the image is a color image or not.
 * \param[out] isMono Whether the image is a mono image or not.
 * \param[out] isBayer Whether the image is a bayer image or not.
 * \param[out] hasAlpha Whether the image has an alpha channel or not.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the parsing succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the parsing failed.
 */
extern "C" bool parseCVTransportFormat(
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
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Convert the `cv` transport parameters into a string to be filled in field
 *        `sensor_msgs::CompressedImage::format`.
 * \param[in] compressionFormat The compression format ("jpeg", "png" etc.).
 * \param[in] rawEncoding Encoding of the raw image (before compression, after decompression).
 * \param[in] compressedEncoding Encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[in] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[in] bitDepth Number of bits used for encoding one raw channel value.
 * \param[in] isColor Whether the image is a color image or not.
 * \param[in] isMono Whether the image is a mono image or not.
 * \param[in] isBayer Whether the image is a bayer image or not.
 * \param[in] hasAlpha Whether the image has an alpha channel or not.
 * \param[in,out] formatAllocator Allocator for the string for the `format` field.
 * \param[in,out] errorStringAllocator Allocator for explanation what failed (used only in case of failure).
 * \return Whether the conversion succeeded. If not, `formatAllocator` is not used, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the conversion failed.
 */
extern "C" bool makeCVTransportFormat(
  const char* compressionFormat,
  const char* rawEncoding,
  const char* compressedEncoding,
  int numChannels,
  int bitDepth,
  bool isColor,
  bool isMono,
  bool isBayer,
  bool hasAlpha,
  cras::allocator_t formatAllocator,
  cras::allocator_t errorStringAllocator
);

/**
 * \brief Create the `cv` transport parameters for the given raw image compressed with the given method.
 * \param[in] imageEncoding `encoding` field of the raw image.
 * \param[in] compressionFormat The compression format ("jpeg", "png", etc.).
 * \param[in,out] compressedEncodingAllocator Allocator for encoding of the compressed image (i.e. `bgr8` for JPEG).
 * \param[out] numChannels Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
 * \param[out] bitDepth Number of bits used for encoding one raw channel value.
 * \param[out] isColor Whether the image is a color image or not.
 * \param[out] isMono Whether the image is a mono image or not.
 * \param[out] isBayer Whether the image is a bayer image or not.
 * \param[out] hasAlpha Whether the image has an alpha channel or not.
 * \param[in,out] errorStringAllocator Allocator for explanation why the parsing failed (used only in case of failure).
 * \return Whether the extraction succeeded. If not, the output parameters are not valid, and the buffer created using
 *         `errorStringAllocator` contains the explanation why the extraction failed.
 */
extern "C" bool extractCVTransportFormat(
  const char* imageEncoding,
  const char* compressionFormat,
  cras::allocator_t compressedEncodingAllocator,
  int& numChannels,
  int& bitDepth,
  bool& isColor,
  bool& isMono,
  bool& isBayer,
  bool& hasAlpha,
  cras::allocator_t errorStringAllocator
);
