<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# OpenCV Image Transport

This package provides ROS image transport "cv" which compresses images using OpenCV `imencode/imdecode` functions.

The `data` field of the `CompressedImage` messages can be directly stored as image files of the corresponding type and interpreted by other programs.

The JPEG and PNG encoding done by this package is the same as `compressed_image_transport`. For better compatibility, use that package if possible.

This image transport also provides the [image_transport_codecs](https://wiki.ros.org/image_transport_codecs) interface.
This means it is possible to directly encode/decode images from C++/Python code without the need for going through
a pub/sub cycle. You don't even need a ROS master for the conversion.

## C/C++ ABI stability

Currently, the package offers encoder options corresponding to `cv::imencode()` in version 4.12 and 5.0.0-alpha.
The related C and C++ structs can change in the future as new `imencode` options will be added.

Make sure when using the C or C++ API directly to always update and rebuild both this and the dependent package at the
same time. Otherwise, memory corruption and segfault errors can happen.

## Build Status

[![CI](https://github.com/ctu-vras/cv_image_transport/actions/workflows/ci.yaml/badge.svg)](https://github.com/ctu-vras/cv_image_transport/actions/workflows/ci.yaml)