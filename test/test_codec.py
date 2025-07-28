#!/usr/bin/env python

# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Unit test for cv_image_transport."""

import os
import struct
import unittest
import sys

import cv2
from rosbag import Bag
from sensor_msgs.msg import CompressedImage, Image

from cv_image_transport import cv_codec
from image_transport_codecs import decode, encode, get_compressed_image_content, CompressedImageContent


def _byte(b):
    if sys.version_info[0] == 2:
        return ord(b)
    return b


def bytes_to_float(b):
    if len(b) == 4:
        return struct.unpack('<f', b)[0]
    return struct.unpack('<%if' % (len(b) / 4,), b)


def float_to_bytes(f):
    if isinstance(f, float):
        return bytes(struct.pack('<f',  f))
    return bytes(struct.pack('<%if' % (len(f),), *f))


class CodecTest(unittest.TestCase):

    def test_cv(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed, err = encode(raw, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "bgr8; jpeg compressed bgr8")

        raw2, err = decode(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(raw.data)):
            self.assertLess(abs(_byte(raw2.data[i]) - _byte(raw.data[i])), 20)

        content, err = get_compressed_image_content(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "jpeg")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "cv", "jp2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_cv_pgm(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "mono8"
        raw.width = raw.height = 2
        raw.step = 2
        raw.data = b'\x00\x64\xc8\xff'

        compressed, err = encode(raw, "cv", {"format": "pgm"})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "mono8; pgm compressed ")

        raw2, err = decode(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        self.assertEqual(raw2.data, raw.data)

        content, err = get_compressed_image_content(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "pgm")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "cv", "jp2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_cv_pbm(self):
        ver = tuple((int(c) for c in cv2.__version__.split(".")))
        if ver[0] < 4 and (ver[0] != 3 or ver[1] != 4 or ver[2] < 1):
            return

        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "mono8"
        raw.width = raw.height = 2
        raw.step = 2
        raw.data = b'\x00\x64\xc8\xff'
        reloaded_data = b'\x00\xff\xff\xff'

        compressed, err = encode(raw, "cv", {"format": "pbm"})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed)
        self.assertIsInstance(compressed, CompressedImage)
        self.assertEqual(compressed.header, raw.header)
        self.assertEqual(compressed.format, "mono8; pbm compressed ")

        raw2, err = decode(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        self.assertEqual(raw2.data, reloaded_data)

        content, err = get_compressed_image_content(compressed, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(content)
        self.assertIsInstance(content, CompressedImageContent)
        self.assertEqual(content.format, "pbm")
        self.assertEqual(content.data, compressed.data)

        content, err = get_compressed_image_content(compressed, "cv", "jp2")
        self.assertEqual(err, "")
        self.assertIsNone(content)

    def test_cv_config(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "bgr8"
        raw.width = raw.height = 2
        raw.step = 6
        raw.data = b'\x00\x00\x00\x64\x64\x64\xc8\xc8\xc8\xff\xff\xff'

        compressed1, err = encode(raw, "cv", {"format": "jpeg", "jpeg_quality": 100})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed1)
        self.assertIsInstance(compressed1, CompressedImage)
        self.assertEqual(compressed1.header, raw.header)
        self.assertEqual(compressed1.format, "bgr8; jpeg compressed bgr8")

        compressed2, err = encode(raw, "cv", {"format": "jpeg", "jpeg_quality": 50})
        self.assertEqual(err, "")
        self.assertIsNotNone(compressed2)
        self.assertIsInstance(compressed2, CompressedImage)
        self.assertEqual(compressed2.header, raw.header)
        self.assertEqual(compressed2.format, "bgr8; jpeg compressed bgr8")

        self.assertGreater(len(compressed1.data), len(compressed2.data))

        raw1, err = decode(compressed1, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw1)
        self.assertIsInstance(raw1, Image)
        self.assertEqual(raw1.header, raw.header)
        self.assertEqual(raw1.step, raw.step)
        self.assertEqual(raw1.width, raw.width)
        self.assertEqual(raw1.height, raw.height)
        self.assertEqual(raw1.encoding, raw.encoding)
        self.assertEqual(raw1.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(raw.data)):
            self.assertLess(abs(_byte(raw1.data[i]) - _byte(raw.data[i])), 20)

        raw2, err = decode(compressed2, "cv")
        self.assertEqual(err, "")
        self.assertIsNotNone(raw2)
        self.assertIsInstance(raw2, Image)
        self.assertEqual(raw2.header, raw.header)
        self.assertEqual(raw2.step, raw.step)
        self.assertEqual(raw2.width, raw.width)
        self.assertEqual(raw2.height, raw.height)
        self.assertEqual(raw2.encoding, raw.encoding)
        self.assertEqual(raw2.is_bigendian, raw.is_bigendian)
        # the color changes a bit after compression and decompression
        for i in range(len(raw.data)):
            self.assertLess(abs(_byte(raw2.data[i]) - _byte(raw.data[i])), 20)

    def test_compressed_wrong_type(self):
        raw = Image()
        raw.header.stamp.secs = 10
        raw.encoding = "32FC1"
        raw.width = raw.height = 2
        raw.step = 8
        raw.data = float_to_bytes((1.0, 2.0, 3.0, 4.0))

        compressed, err = encode(raw, "cv")
        self.assertNotEqual(err, "")
        self.assertIsNone(compressed)

    def test_bag(self):
        d = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data")
        raw_bag = os.path.join(d, "raw.bag")
        compressed_bag = os.path.join(d, "compressed.bag")

        with Bag(raw_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                msg.is_bigendian = 0  # Spot driver sets some images to 1 for some reason
                if topic == "/spot/camera/hand_color/image":
                    hand_color_raw = msg

        with Bag(compressed_bag) as bag:
            for topic, msg, _ in bag.read_messages():
                msg.header.seq = 0  # seq number might differ, so we zero it out
                if topic == "/spot/camera/hand_color/image/cv":
                    hand_color_compressed = msg

        # Hand color

        for it in range(3):  # Test several iterations
            compressed, err = encode(hand_color_raw, "cv")
            self.assertEqual(err, "")
            self.assertIsNotNone(compressed)
            self.assertIsInstance(compressed, CompressedImage)
            self.assertEqual(compressed.header.stamp, hand_color_compressed.header.stamp)
            self.assertEqual(compressed.header.frame_id, hand_color_compressed.header.frame_id)
            self.assertEqual(compressed.format, hand_color_compressed.format)
            self.assertEqual(compressed.data, hand_color_compressed.data)

            raw2, err = decode(compressed, "cv")
            self.assertEqual(err, "")
            self.assertIsNotNone(raw2)
            self.assertIsInstance(raw2, Image)
            self.assertEqual(raw2.header, hand_color_raw.header)
            self.assertEqual(raw2.step, hand_color_raw.step)
            self.assertEqual(raw2.width, hand_color_raw.width)
            self.assertEqual(raw2.height, hand_color_raw.height)
            self.assertEqual(raw2.encoding, hand_color_raw.encoding)
            self.assertEqual(raw2.is_bigendian, hand_color_raw.is_bigendian)

            # JPEG compression changes this image quite a lot, so we examine the error histogram.
            # It is a 1920x1080 image, so having 7000 pixels with color difference of 20-30 is quite okay.
            err20 = 0
            err30 = 0
            err40 = 0
            err50 = 0
            err80 = 0
            err = 0
            for i in range(len(hand_color_raw.data)):
                e = abs(_byte(raw2.data[i]) - _byte(hand_color_raw.data[i]))
                if e < 20:
                    err20 += 1
                elif e < 30:
                    err30 += 1
                elif e < 40:
                    err40 += 1
                elif e < 50:
                    err50 += 1
                elif e < 80:
                    err80 += 1
                else:
                    err += 1
            # err20 is ok in any amount
            self.assertLessEqual(err30, 7000)
            self.assertLessEqual(err40, 500)
            self.assertLessEqual(err50, 100)
            self.assertLessEqual(err80, 30)
            self.assertLessEqual(0, err)


if __name__ == '__main__':
    unittest.main()
