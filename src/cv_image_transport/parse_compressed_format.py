# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Methods for parsing the values of field `sensor_msgs::CompressedImage::format` for `cv` codec."""

from collections import namedtuple
from ctypes import RTLD_GLOBAL, c_bool, c_uint32, c_uint8, c_char_p, POINTER, byref
from enum import Enum

from cv_image_transport.cfg import CVPublisherConfig
from sensor_msgs.msg import CompressedImage, Image

from cras.ctypes_utils import get_ro_c_buffer, load_library, Allocator, StringAllocator


class CVTransportCompressionFormat(Enum):
    """Compression format of `cv` codec"""

    JPEG = CVPublisherConfig.CVPublisher_format_jpeg
    """JPEG compression"""

    PNG = CVPublisherConfig.CVPublisher_format_png
    """PNG compression"""

    TIF = CVPublisherConfig.CVPublisher_format_tif
    """TIF compression"""

    BMP = CVPublisherConfig.CVPublisher_format_bmp
    """BMP compression"""

    DIB = CVPublisherConfig.CVPublisher_format_dib
    """DIB compression"""

    JP2 = CVPublisherConfig.CVPublisher_format_jp2
    """JP2 compression"""

    JXL = CVPublisherConfig.CVPublisher_format_jxl
    """JXL compression"""

    PBM = CVPublisherConfig.CVPublisher_format_pbm
    """PBM compression"""

    PGM = CVPublisherConfig.CVPublisher_format_pgm
    """PGM compression"""

    PPM = CVPublisherConfig.CVPublisher_format_ppm
    """PPM compression"""

    PXM = CVPublisherConfig.CVPublisher_format_pxm
    """PXM compression"""

    PNM = CVPublisherConfig.CVPublisher_format_pnm
    """PNM compression"""

    PAM = CVPublisherConfig.CVPublisher_format_pam
    """PAM compression"""

    SR = CVPublisherConfig.CVPublisher_format_sr
    """SR compression"""

    RAS = CVPublisherConfig.CVPublisher_format_ras
    """RAS compression"""

    GIF = CVPublisherConfig.CVPublisher_format_gif
    """GIF compression"""

    WEBP = CVPublisherConfig.CVPublisher_format_webp
    """WEBP compression"""

    AVIF = CVPublisherConfig.CVPublisher_format_avif
    """AVIF compression"""

    EXR = CVPublisherConfig.CVPublisher_format_exr
    """EXR compression"""

    HDR = CVPublisherConfig.CVPublisher_format_hdr
    """HDR compression"""

    PIC = CVPublisherConfig.CVPublisher_format_pic
    """PIC compression"""


class CVTransportFormat(namedtuple(
    "CVTransportFormat",
        ("format", "format_string", "raw_encoding", "compressed_encoding", "num_channels", "bit_depth", "is_color",
         "is_mono", "is_bayer", "has_alpha"))):
    """Decoded meaning of field `sensor_msgs::CompressedImage::format` for `cv` transport.

    :param CVTransportCompressionFormat format: The compression format (JPEG/PNG/etc.).
    :param str format_string: Text version of the compression format ("jpeg"/"png"/etc.).
    :param str rawEncoding: Encoding of the raw image (before compression, after decompression).
    :param str compressedEncoding: Encoding of the compressed image (i.e. `bgr8` for JPEG).
    :param int numChannels: Number of channels of the raw image data (1 for mono/depth images, 3-4 for color).
    :param int bitDepth: Number of bits used for encoding one raw channel value.
    :param bool isColor: Whether the image is a color image or not.
    :param bool isMono: Whether the image is a mono image or not.
    :param bool isBayer: Whether the image is a Bayer image or not.
    :param bool hasAlpha: Whether the image has alpha channel or not.
    """
    __slots__ = ()  # prevent adding more fields


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        __codec = load_library('cv_image_transport', mode=RTLD_GLOBAL)
        if __codec is None:
            return None

        # Add function signatures

        __codec.parseCVTransportFormat.restype = c_bool
        __codec.parseCVTransportFormat.argtypes = [
            c_char_p,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            POINTER(c_uint32), POINTER(c_uint32), POINTER(c_bool), POINTER(c_bool), POINTER(c_bool), POINTER(c_bool),
            Allocator.ALLOCATOR,
        ]

        __codec.makeCVTransportFormat.restype = c_bool
        __codec.makeCVTransportFormat.argtypes = [
            c_char_p, c_char_p, c_char_p, c_uint32, c_uint32, c_bool, c_bool, c_bool, c_bool,
            Allocator.ALLOCATOR,
            Allocator.ALLOCATOR,
        ]

        __codec.extractCVTransportFormat.restype = c_bool
        __codec.extractCVTransportFormat.argtypes = [
            c_char_p, c_char_p,
            Allocator.ALLOCATOR, POINTER(c_uint32), POINTER(c_uint32),
            POINTER(c_bool), POINTER(c_bool), POINTER(c_bool), POINTER(c_bool),
            Allocator.ALLOCATOR,
        ]

    return __codec


def parse_cv_transport_format(format):
    """Parse the string from field `sensor_msgs::CompressedImage::format` using `cv` transport into
    :py:class:`CVTransportFormat` structure.

    :param format: The `format` field text, or a :sensor_msgs:`CompressedImage`.
    :type format: str or sensor_msgs.msg.CompressedImage
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CVTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(format, CompressedImage):
        format = format.format

    compression_format_allocator = StringAllocator()
    raw_encoding_allocator = StringAllocator()
    compressed_encoding_allocator = StringAllocator()
    error_allocator = StringAllocator()

    num_channels = c_uint32()
    bit_depth = c_uint32()
    is_color = c_bool()
    is_mono = c_bool()
    is_bayer = c_bool()
    has_alpha = c_bool()

    args = [
        format.encode("utf-8"),
        compression_format_allocator.get_cfunc(), raw_encoding_allocator.get_cfunc(),
        compressed_encoding_allocator.get_cfunc(), byref(num_channels), byref(bit_depth),
        byref(is_color), byref(is_mono), byref(is_bayer), byref(has_alpha),
        error_allocator.get_cfunc(),
    ]

    ret = codec.parseCVTransportFormat(*args)

    if ret:
        return CVTransportFormat(
            CVTransportCompressionFormat(compression_format_allocator.value),
            compression_format_allocator.value, raw_encoding_allocator.value, compressed_encoding_allocator.value,
            int(num_channels.value), int(bit_depth.value),
            is_color.value, is_mono.value, is_bayer.value, has_alpha.value), ""
    return None, error_allocator.value


def make_cv_transport_format(format):
    """Convert the :py:class:`CompressedTransportFormat` structure into a string to be filled in field
    `sensor_msgs::CompressedImage::format` of `cv` transport image.

    :param CVTransportFormat format: The format to convert.
    :return: The string for the `format` field.
    :rtype: (str or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    format_allocator = StringAllocator()
    error_allocator = StringAllocator()

    args = [
        format.format_string.encode("utf-8"), format.raw_encoding.encode("utf-8"),
        format.compressed_encoding.encode("utf-8"), format.num_channels, format.bit_depth,
        format.is_color, format.is_mono, format.is_bayer, format.has_alpha,
        format_allocator.get_cfunc(),
        error_allocator.get_cfunc(),
    ]

    ret = codec.makeCVTransportFormat(*args)

    if ret:
        return format_allocator.value, ""
    return None, error_allocator.value


def extract_cv_transport_format(image_encoding, compression_format):
    """Create the :py:class:`CompressedTransportFormat` structure for the given raw image compressed with the given
    method.

    :param image_encoding: `encoding` field of the raw image, or a :sensor_msgs:`Image`.
    :type image_encoding: str or sensor_msgs.msg.Image
    :param CompressedTransportCompressionFormat compression_format: The target compression method.
    :return: Tuple of the parsed structure or error string. If the parsing fails, structure is `None` and error string
             is filled.
    :rtype: (CVTransportFormat or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    if isinstance(image_encoding, Image):
        image_encoding = image_encoding.encoding

    compressed_encoding_allocator = StringAllocator()
    error_allocator = StringAllocator()

    num_channels = c_uint32()
    bit_depth = c_uint32()
    is_color = c_bool()
    is_mono = c_bool()
    is_bayer = c_bool()
    has_alpha = c_bool()

    args = [
        image_encoding.encode("utf-8"), compression_format.value.encode("utf-8"),
        compressed_encoding_allocator.get_cfunc(), byref(num_channels), byref(bit_depth),
        byref(is_color), byref(is_mono), byref(is_bayer), byref(has_alpha),
        error_allocator.get_cfunc(),
    ]

    ret = codec.extractCVTransportFormat(*args)

    if ret:
        return CVTransportFormat(
            compression_format, compression_format.value, image_encoding, compressed_encoding_allocator.value,
            int(num_channels.value), int(bit_depth.value),
            is_color.value, is_mono.value, is_bayer.value, has_alpha.value), ""
    return None, error_allocator.value


if __name__ == '__main__':
    def main():
        print(parse_cv_transport_format("bgr8; jpeg compressed bgr8"))
        print(parse_cv_transport_format(CompressedImage(format="bgr8; jpeg compressed bgr8")))
        print(extract_cv_transport_format("bgr8", CVTransportCompressionFormat.JPEG))
        print(extract_cv_transport_format(Image(encoding="bgr8"), CVTransportCompressionFormat.JPEG))
        print(make_cv_transport_format(CVTransportFormat(
            CVTransportCompressionFormat.JPEG, "jpeg", "bgr8", "bgr8", 3, 8, True, False, False, False)))

        print(parse_cv_transport_format("mono8; png compressed "))
        print(parse_cv_transport_format(CompressedImage(format="mono8; png compressed ")))
        print(extract_cv_transport_format("mono8", CVTransportCompressionFormat.PNG))
        print(extract_cv_transport_format(Image(encoding="mono8"), CVTransportCompressionFormat.PNG))
        print(make_cv_transport_format(CVTransportFormat(
            CVTransportCompressionFormat.PNG, "png", "mono8", "mono8", 1, 8, False, True, False, False)))

        print(parse_cv_transport_format("mono8; pbm compressed "))
        print(parse_cv_transport_format(CompressedImage(format="mono8; pbm compressed ")))
        print(extract_cv_transport_format("mono8", CVTransportCompressionFormat.PBM))
        print(extract_cv_transport_format(Image(encoding="mono8"), CVTransportCompressionFormat.PBM))
        print(make_cv_transport_format(CVTransportFormat(
            CVTransportCompressionFormat.PBM, "pbm", "mono8", "mono8", 1, 8, False, True, False, False)))

        print(parse_cv_transport_format("16UC1; png compressed "))
        print(parse_cv_transport_format(CompressedImage(format="16UC1; png compressed ")))
        print(extract_cv_transport_format("16UC1", CVTransportCompressionFormat.PNG))
        print(extract_cv_transport_format(Image(encoding="16UC1"), CVTransportCompressionFormat.PNG))
        print(make_cv_transport_format(CVTransportFormat(
            CVTransportCompressionFormat.PNG, "png", "16UC1", "16UC1", 1, 16, False, True, False, False)))

        print(parse_cv_transport_format("invalid"))

    main()
