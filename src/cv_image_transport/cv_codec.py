# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of images compressed with the 'cv' transport."""

from ctypes import RTLD_GLOBAL, c_bool, c_int, c_int32, c_uint8, c_uint32, c_char_p, c_size_t, POINTER, byref, Structure
import time

from cv_image_transport.cfg import CVPublisherConfig
from sensor_msgs.msg import CompressedImage, Image

from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, \
    get_ro_c_buffer


class _CVPublisherConfig(Structure):
    _fields_ = [
        ('format', c_char_p),
        ('enable', c_int),
        ('jpeg_quality', c_bool),
        ('jpeg_progressive', c_int),
        ('jpeg_optimize', c_bool),
        ('jpeg_rst_interval', c_bool),
        ('jpeg_luma_quality', c_int),
        ('jpeg_chroma_quality', c_int),
        ('jpeg_sampling_factor', c_int),
        ('png_level', c_int),
        ('png_strategy', c_int),
        ('png_bilevel', c_int),
        ('png_filter', c_bool),
        ('png_zlib_buffer_size', c_int),
        ('pxm_binary', c_int),
        ('exr_type', c_bool),
        ('exr_compression', c_int),
        ('exr_dwa_compression_level', c_int),
        ('webp_quality', c_int),
        ('hdr_compression', c_int),
        ('pam_tupletype', c_int),
        ('tiff_resunit', c_int),
        ('tiff_xdpi', c_int),
        ('tiff_ydpi', c_int),
        ('tiff_compression', c_int),
        ('tiff_rows_per_strip', c_int),
        ('tiff_predictor', c_int),
        ('jpeg2000_compression_x1000', c_int),
        ('avif_quality', c_int),
        ('avif_depth', c_int),
        ('avif_speed', c_int),
        ('jpegxl_quality', c_int),
        ('jpegxl_effort', c_int),
        ('jpegxl_distance', c_int),
        ('jpegxl_decoding_speed', c_int),
        ('gif_quality', c_int),
        ('gif_dither', c_int),
        ('gif_transparency', c_int),
        ('gif_colortable', c_int),
    ]


__codec = None


def __get_library():
    global __codec
    if __codec is None:
        if load_library('cv_image_transport', mode=RTLD_GLOBAL) is not None:
            __codec = load_library('cv_codec')
            if __codec is None:
                return None

            # Add function signatures

            __codec.cvConfigToCVOpts.restype = None
            __codec.cvConfigToCVOpts.argtypes = [
                POINTER(_CVPublisherConfig), Allocator.ALLOCATOR,
            ]

            __codec.cvCodecEncode.restype = c_bool
            __codec.cvCodecEncode.argtypes = [
                c_uint32, c_uint32, c_char_p, c_uint8, c_uint32, c_size_t, POINTER(c_uint8),
                POINTER(_CVPublisherConfig),
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR
            ]

            __codec.cvCodecEncodeWithCVOpts.restype = c_bool
            __codec.cvCodecEncodeWithCVOpts.argtypes = [
                c_uint32, c_uint32, c_char_p, c_uint8, c_uint32, c_size_t, POINTER(c_uint8),
                c_char_p, c_size_t, POINTER(c_int32),
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR
            ]

            __codec.cvCodecDecode.restype = c_bool
            __codec.cvCodecDecode.argtypes = [
                c_char_p, c_size_t, POINTER(c_uint8),
                POINTER(c_uint32), POINTER(c_uint32), Allocator.ALLOCATOR, POINTER(c_uint8), POINTER(c_uint32),
                Allocator.ALLOCATOR,
                Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            ]

    return __codec


class _CVOptsAllocator(Allocator):
    """ctypes allocator suitable for allocating int32 arrays."""

    def _alloc(self, size):
        return (c_int32 * size)()

    @property
    def value(self):
        if len(self.allocated) == 0:
            return None
        return self.allocated[0]

    @property
    def values(self):
        return self.allocated


def cv_config_dict_to_c_struct(config=None):
    """Convert the given CVPublisherConfig Python dict to _CVPublisherConfig ctypes struct.

    :param dict config: The encoder config.
    :return: The ctypes config.
    :rtype: _CVPublisherConfig
    """
    c_config = _CVPublisherConfig()
    for name, value in CVPublisherConfig.defaults.items():
        if hasattr(c_config, name):
            if isinstance(value, str):
                value = value.encode("utf-8")
            setattr(c_config, name, value)
    if config is None:
        return c_config

    for name in config:
        if hasattr(c_config, name):
            value = config[name]
            if isinstance(value, str):
                value = value.encode("utf-8")
            setattr(c_config, name, value)

    return c_config


def cv_config_to_cv_opts(config=None):
    """Convert the given encoder config to CV imencode options.

    :param config: The encoder config.
    :type config: _CVPublisherConfig or dict
    :return: The parsed CV options.
    :rtype: list
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load cv codec library."

    if config is None:
        return [], ""

    if isinstance(config, _CVPublisherConfig):
        c_config = config
    elif isinstance(config, dict):
        c_config = cv_config_dict_to_c_struct(config)
    else:
        return None, "config has to be a dict or cv_image_transport_CVPublisherConfig instance."

    cv_opts_allocator = _CVOptsAllocator()

    codec.cvConfigToCVOpts(byref(c_config), cv_opts_allocator.get_cfunc())
    return cv_opts_allocator.value, ""


def encode(raw, config=None):
    """Encode the given raw image into a :sensor_msgs:`CompressedImage` with "cv" codec.

    :param sensor_msgs.msg.Image raw: The raw image.
    :param dict config: Configuration of the encoding process. You can use the same values as those offered by dynamic
                        reconfigure of the :roswiki:`cv_image_transport` publisher.
    :return: Tuple of compressed image and error string. If the compression fails (e.g. wrong image dimensions or bit
             depth), the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.CompressedImage or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load cv codec library."

    format_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    c_config = cv_config_dict_to_c_struct(config)

    args = [
        raw.height, raw.width, raw.encoding.encode("utf-8"), raw.is_bigendian, raw.step, len(raw.data),
        get_ro_c_buffer(raw.data), byref(c_config),
        format_allocator.get_cfunc(), data_allocator.get_cfunc(), error_allocator.get_cfunc(), log_allocator.get_cfunc()
    ]
    ret = codec.cvCodecEncode(*args)

    log_allocator.print_log_messages()
    if ret:
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format = format_allocator.value
        compressed.data = data_allocator.value
        return compressed, ""
    return None, error_allocator.value


def encode_with_cv_opts(raw, format, cv_opts=None):
    """Encode the given raw image into a :sensor_msgs:`CompressedImage` with "cv" codec.

    :param sensor_msgs.msg.Image raw: The raw image.
    :param str format: The output format of the image (i.e. image format file extension).
    :param list cv_opts: OpenCV imencode options.
    :return: Tuple of compressed image and error string. If the compression fails (e.g. wrong image dimensions or bit
             depth), the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.CompressedImage or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load cv codec library."

    format_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    if cv_opts is None:
        cv_opts = []

    args = [
        raw.height, raw.width, raw.encoding.encode("utf-8"), raw.is_bigendian, raw.step, len(raw.data),
        get_ro_c_buffer(raw.data), format.encode("utf-8"), len(cv_opts), (c_int32 * len(cv_opts))(*cv_opts),
        format_allocator.get_cfunc(), data_allocator.get_cfunc(), error_allocator.get_cfunc(), log_allocator.get_cfunc()
    ]
    ret = codec.cvCodecEncodeWithCVOpts(*args)

    log_allocator.print_log_messages()
    if ret:
        compressed = CompressedImage()
        compressed.header = raw.header
        compressed.format = format_allocator.value
        compressed.data = data_allocator.value
        return compressed, ""
    return None, error_allocator.value


def decode(compressed, config=None):
    """Decode the given :sensor_msgs:`CompressedImage` encoded with "cv" codec into a raw image.

    :param sensor_msgs.msg.CompressedImage compressed: The compressed image.
    :param dict config: Not used.
    :return: Tuple of raw image and error string. If decoding failed, the image is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.Image or None, str)
    """
    codec = __get_library()
    if codec is None:
        return None, "Could not load the codec library."

    encoding_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    raw_height = c_uint32()
    raw_width = c_uint32()
    raw_is_big_endian = c_uint8()
    raw_step = c_uint32()

    args = [
        compressed.format.encode("utf-8"), len(compressed.data), get_ro_c_buffer(compressed.data),
        byref(raw_height), byref(raw_width), encoding_allocator.get_cfunc(), byref(raw_is_big_endian), byref(raw_step),
        data_allocator.get_cfunc(),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]

    ret = codec.cvCodecDecode(*args)

    log_allocator.print_log_messages()
    if ret:
        raw = Image()
        raw.header = compressed.header
        raw.height = raw_height.value
        raw.width = raw_width.value
        raw.encoding = encoding_allocator.value
        raw.is_bigendian = raw_is_big_endian.value
        raw.step = raw_step.value
        raw.data = data_allocator.value
        return raw, ""
    return None, error_allocator.value


if __name__ == '__main__':
    def main():
        import rospy
        raw = Image()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.encoding = 'bgr8'
        raw.step = 6
        raw.width = raw.height = 2
        raw.data = [0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255]

        rospy.init_node("test")
        pub = rospy.Publisher("test/cv", CompressedImage, queue_size=1, latch=True)
        pub2 = rospy.Publisher("test", Image, queue_size=1, latch=True)
        pub3 = rospy.Publisher("test2/cv", CompressedImage, queue_size=1, latch=True)
        time.sleep(1)

        compressed, err = encode(raw, {})

        print(bool(compressed), err)
        if compressed is not None:
            pub.publish(compressed)

        raw, err = decode(compressed)

        print(bool(raw), err)
        if raw is not None:
            pub2.publish(raw)

        compressed, err = encode_with_cv_opts(raw, "pnm", [])

        print(bool(compressed), err)
        if compressed is not None:
            pub3.publish(compressed)

        rospy.spin()

    main()
