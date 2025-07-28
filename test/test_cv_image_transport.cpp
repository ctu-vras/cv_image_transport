// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for cv_image_transport.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>
#include <cras_cpp_common/log_utils/macros.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

TEST(CvImageTransport, Roundtrip)  // NOLINT
{
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

  sensor_msgs::CompressedImageConstPtr receivedCvMsg;
  sensor_msgs::ImageConstPtr receivedRawMsg;

  ros::NodeHandle nh;
  auto rawPub = nh.advertise<sensor_msgs::Image>("/spot/camera/hand_color/image", 1);
  auto cvSub = nh.subscribe<sensor_msgs::CompressedImage>("/republished/cv", 1,
    [&](const sensor_msgs::CompressedImageConstPtr& msg)
    {
      receivedCvMsg = msg;
    });
  auto raw2Sub = nh.subscribe<sensor_msgs::Image>("/republished2", 1,
    [&](const sensor_msgs::ImageConstPtr& msg)
    {
      receivedRawMsg = msg;
    });

  for (size_t i = 0; i < 100; i++)
  {
    if (rawPub.getNumSubscribers() > 0 && cvSub.getNumPublishers() > 0 && raw2Sub.getNumPublishers() > 0)
      break;
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for subscribers and publishers");
    ros::WallDuration(0.1).sleep();
  }

  for (size_t i = 0; i < 100; i++)
  {
    rawPub.publish(handColorRaw);
    ros::spinOnce();
    ROS_WARN_DELAYED_THROTTLE(1.0, "Waiting for messages");
    ros::WallDuration(0.1).sleep();
    if (receivedCvMsg != nullptr && receivedRawMsg != nullptr)
      break;
  }

  ASSERT_NE(nullptr, receivedCvMsg);
  ASSERT_NE(nullptr, receivedRawMsg);

  EXPECT_EQ(handColorCompressed.header.stamp, receivedCvMsg->header.stamp);
  EXPECT_EQ(handColorCompressed.header.frame_id, receivedCvMsg->header.frame_id);
  EXPECT_EQ(handColorCompressed.format, receivedCvMsg->format);
  EXPECT_EQ(handColorCompressed.data.size(), receivedCvMsg->data.size());
  // The binary representation of the JPEG might differ a bit, but the binary size should roughly match
  EXPECT_NEAR(handColorCompressed.data.size(), receivedCvMsg->data.size(), 0.01 * receivedCvMsg->data.size());

  EXPECT_EQ(handColorRaw.header.stamp, receivedRawMsg->header.stamp);
  EXPECT_EQ(handColorRaw.header.frame_id, receivedRawMsg->header.frame_id);
  EXPECT_EQ(handColorRaw.step, receivedRawMsg->step);
  EXPECT_EQ(handColorRaw.width, receivedRawMsg->width);
  EXPECT_EQ(handColorRaw.height, receivedRawMsg->height);
  EXPECT_EQ(handColorRaw.encoding, receivedRawMsg->encoding);
  EXPECT_EQ(handColorRaw.is_bigendian, receivedRawMsg->is_bigendian);
  ASSERT_EQ(handColorRaw.data.size(), receivedRawMsg->data.size());

  // JPEG compression changes this image quite a lot, so we examine the error histogram.
  // It is a 1920x1080 image, so having 7000 pixels with color difference of 20-30 is quite okay.

  size_t err20 = 0, err30 = 0, err40 = 0, err50 = 0, err80 = 0, err = 0;
  for (size_t j = 0; j < receivedRawMsg->data.size(); ++j)
  {
    const auto e = fabs(handColorRaw.data[j] - receivedRawMsg->data[j]);
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_cv_image_transport");

  ros::NodeHandle nh;  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
