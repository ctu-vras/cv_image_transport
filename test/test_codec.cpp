// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for cv_image_transport.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <cras_cpp_common/string_utils.hpp>
#include <image_transport_codecs/image_transport_codecs.h>
#include <cv_image_transport/cv_codec.h>

using namespace image_transport_codecs;  // NOLINT(build/namespaces)
using namespace cv_image_transport;  // NOLINT(build/namespaces)

TEST(CvImageTransport, Cv)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  const auto compressedShifter = codecs.encode(raw, "cv");
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "cv");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    // the color changes a bit after compression and decompression
    EXPECT_LT(fabs(raw2->data[i] - raw.data[i]), 20);
  }

  auto content = codecs.getCompressedImageContent(compressedShifter.value(), "cv");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("jpeg", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "jpeg");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("jpeg", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "foo");
  ASSERT_TRUE(content);
  EXPECT_FALSE(content->has_value());
}

TEST(CvImageTransport, CvPGM)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "mono8";
  raw.width = raw.height = 2;
  raw.step = 2;
  raw.data = {0, 100, 200, 255};

  CVPublisherConfig config(CVPublisherConfig::__getDefault__());
  config.format = "pgm";
  config.pxm_binary = true;

  const auto compressedShifter = codecs.encode(raw, "cv", config);
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("mono8; pgm compressed ", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "cv");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  EXPECT_EQ(raw2->data, raw.data);

  auto content = codecs.getCompressedImageContent(compressedShifter.value(), "cv");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("pgm", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "pgm");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("pgm", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "foo");
  ASSERT_TRUE(content);
  EXPECT_FALSE(content->has_value());
}

// Proper PBM support was added in OpenCV 3.4.1; before, it was identical to PGM.
#if CV_VERSION_MAJOR >= 4 || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR == 4 && CV_VERSION_REVISION >= 1)
TEST(CvImageTransport, CvPBM)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "mono8";
  raw.width = raw.height = 2;
  raw.step = 2;
  raw.data = {0, 100, 200, 255};
  // PBM saves only 0 or 1, so when decoded back to mono8, the data differ
  std::vector<uint8_t> reloadedData = {0, 255, 255, 255};

  CVPublisherConfig config(CVPublisherConfig::__getDefault__());
  config.format = "pbm";
  config.pxm_binary = true;

  const auto compressedShifter = codecs.encode(raw, "cv", config);
  ASSERT_TRUE(compressedShifter);
  ASSERT_NE("", compressedShifter->getDataType());
  ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed->header, raw.header);
  EXPECT_EQ("mono8; pbm compressed ", compressed->format);

  const auto raw2 = codecs.decode(compressedShifter.value(), "cv");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  EXPECT_EQ(raw2->data, reloadedData);

  auto content = codecs.getCompressedImageContent(compressedShifter.value(), "cv");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("pbm", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "pbm");
  ASSERT_TRUE(content);
  ASSERT_TRUE(content->has_value());
  EXPECT_EQ("pbm", (*content)->format);
  EXPECT_EQ(compressed->data, (*content)->data);

  content = codecs.getCompressedImageContent(compressedShifter.value(), "cv", "foo");
  ASSERT_TRUE(content);
  EXPECT_FALSE(content->has_value());
}
#endif

TEST(CvImageTransport, CvConfig)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "bgr8";
  raw.width = raw.height = 2;
  raw.step = 6;
  raw.data = {0, 0, 0, 100, 100, 100, 200, 200, 200, 255, 255, 255};

  XmlRpc::XmlRpcValue config1;
  config1["format"] = CVPublisher_format_jpeg;
  config1["jpeg_quality"] = 100;
  const auto compressedShifter1 = codecs.encode(raw, "cv", config1);
  ASSERT_TRUE(compressedShifter1);
  ASSERT_NE("", compressedShifter1->getDataType());
  ASSERT_NO_THROW(compressedShifter1->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed1 = compressedShifter1->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed1->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed1->format);

  XmlRpc::XmlRpcValue config2;
  config2["format"] = CVPublisher_format_jpeg;
  config2["jpeg_quality"] = 50;
  const auto compressedShifter2 = codecs.encode(raw, "cv", config2);
  ASSERT_TRUE(compressedShifter2);
  ASSERT_NE("", compressedShifter2->getDataType());
  ASSERT_NO_THROW(compressedShifter2->instantiate<sensor_msgs::CompressedImage>());
  const auto& compressed2 = compressedShifter2->instantiate<sensor_msgs::CompressedImage>();
  EXPECT_EQ(compressed2->header, raw.header);
  EXPECT_EQ("bgr8; jpeg compressed bgr8", compressed2->format);

  EXPECT_GT(compressed1->data.size(), compressed2->data.size());

  const auto raw1 = codecs.decode(compressedShifter1.value(), "cv");
  ASSERT_TRUE(raw1);

  EXPECT_EQ(raw1->header, raw.header);
  EXPECT_EQ(raw1->step, raw.step);
  EXPECT_EQ(raw1->width, raw.width);
  EXPECT_EQ(raw1->height, raw.height);
  EXPECT_EQ(raw1->encoding, raw.encoding);
  EXPECT_EQ(raw1->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    // the color changes a bit after compression and decompression
    EXPECT_LT(fabs(raw1->data[i] - raw.data[i]), 20);
  }

  const auto raw2 = codecs.decode(compressedShifter2.value(), "cv");
  ASSERT_TRUE(raw2);

  EXPECT_EQ(raw2->header, raw.header);
  EXPECT_EQ(raw2->step, raw.step);
  EXPECT_EQ(raw2->width, raw.width);
  EXPECT_EQ(raw2->height, raw.height);
  EXPECT_EQ(raw2->encoding, raw.encoding);
  EXPECT_EQ(raw2->is_bigendian, raw.is_bigendian);
  for (size_t i = 0; i < raw.data.size(); ++i)
  {
    // the color changes a bit after compression and decompression
    EXPECT_LT(fabs(raw2->data[i] - raw.data[i]), 20);
  }
}

TEST(CvImageTransport, CompressedWrongType)
{
  ImageTransportCodecs codecs;

  sensor_msgs::Image raw;
  raw.header.stamp.sec = 10;
  raw.encoding = "32FC1";
  raw.width = raw.height = 2;
  raw.step = 8;
  float floats[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  auto bytes = reinterpret_cast<uint8_t*>(floats);
  raw.data.resize(16);
  memcpy(&raw.data[0], bytes, 16);

  const auto compressedShifter = codecs.encode(raw, "cv");
  ASSERT_FALSE(compressedShifter);
  ASSERT_NE("", compressedShifter.error());
}

TEST(CvImageTransport, Bag)
{
  ImageTransportCodecs codecs;

  rosbag::Bag rawBag(std::string(TEST_DATA_DIR) + "/raw.bag");
  rosbag::Bag compressedBag(std::string(TEST_DATA_DIR) + "/compressed.bag");

  sensor_msgs::Image handColorRaw;

  sensor_msgs::CompressedImage handColorCompressed;

  for (const auto& data : rosbag::View(rawBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::Image>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    msgPtr->is_bigendian = 0;  // Spot driver sets some images to 1 for some reason
    if (data.getTopic() == "/spot/camera/hand_color/image")
      handColorRaw = *msgPtr;
  }

  for (const auto& data : rosbag::View(compressedBag))
  {
    auto msgPtr = data.instantiate<sensor_msgs::CompressedImage>();
    ASSERT_NE(nullptr, msgPtr);
    msgPtr->header.seq = 0;  // seq number might differ, so we zero it out
    if (data.getTopic() == "/spot/camera/hand_color/image/cv")
      handColorCompressed = *msgPtr;
  }

  for (size_t i = 0; i < 3; ++i)  // test several iterations
  {
    const auto compressedShifter = codecs.encode(handColorRaw, "cv");
    ASSERT_TRUE(compressedShifter);
    ASSERT_NO_THROW(compressedShifter->instantiate<sensor_msgs::CompressedImage>());
    const auto compressed = compressedShifter->instantiate<sensor_msgs::CompressedImage>();
    EXPECT_EQ(handColorCompressed.header, compressed->header);
    EXPECT_EQ(handColorCompressed.format, compressed->format);
    EXPECT_EQ(handColorCompressed.data.size(), compressed->data.size());
    // The binary representation of the JPEG might differ a bit, but the binary size should roughly match
    EXPECT_NEAR(handColorCompressed.data.size(), compressed->data.size(), 0.01 * compressed->data.size());

    const auto rawImg = codecs.decodeTyped(*compressed, "cv");
    ASSERT_TRUE(rawImg);
    EXPECT_EQ(handColorRaw.header, rawImg->header);
    EXPECT_EQ(handColorRaw.step, rawImg->step);
    EXPECT_EQ(handColorRaw.width, rawImg->width);
    EXPECT_EQ(handColorRaw.height, rawImg->height);
    EXPECT_EQ(handColorRaw.encoding, rawImg->encoding);
    EXPECT_EQ(handColorRaw.is_bigendian, rawImg->is_bigendian);
    ASSERT_EQ(handColorRaw.data.size(), rawImg->data.size());

    // JPEG compression changes this image quite a lot, so we examine the error histogram.
    // It is a 1920x1080 image, so having 7000 pixels with color difference of 20-30 is quite okay.

    size_t err20 = 0, err30 = 0, err40 = 0, err50 = 0, err80 = 0, err = 0;
    for (size_t j = 0; j < rawImg->data.size(); ++j)
    {
      const auto e = fabs(handColorRaw.data[j] - rawImg->data[j]);
      if (e < 20) err20++;
      else if (e < 30) err30++;
      else if (e < 40) err40++;
      else if (e < 50) err50++;
      else if (e < 80) err80++;
      else err++;
    }
    // err20 is ok in any amount
    EXPECT_LT(err30, 7000);
    EXPECT_LT(err40, 500);
    EXPECT_LT(err50, 100);
    EXPECT_LT(err80, 30);
    EXPECT_EQ(0, err);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
