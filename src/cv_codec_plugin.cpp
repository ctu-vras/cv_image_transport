// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for cv transport codec.
 * \author Martin Pecka
 */

#include <pluginlib/class_list_macros.h>

#include <image_transport_codecs/image_transport_codec_plugin.h>
#include <cv_image_transport/cv_codec.h>

namespace cv_image_transport
{

class CVCodecPlugin : public image_transport_codecs::ImageTransportCodecPluginBase<CVCodec> {};

}

PLUGINLIB_EXPORT_CLASS(cv_image_transport::CVCodecPlugin, image_transport_codecs::ImageTransportCodecPlugin)
