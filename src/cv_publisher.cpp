// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Plugin for cv transport codec.
 * \author Martin Pecka
 */

#include <functional>
#include <memory>
#include <string>

#include <cv_image_transport/cv_codec.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/publisher_plugin.h>
#include <image_transport/simple_publisher_plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/CompressedImage.h>

namespace cv_image_transport
{

class Publisher final : public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
{
public:
  ~Publisher() override = default;

  std::string getTransportName() const override
  {
    return this->codec.getTransportName();
  }

protected:
  void advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
    const image_transport::SubscriberStatusCallback& user_connect_cb,
    const image_transport::SubscriberStatusCallback& user_disconnect_cb, const ros::VoidPtr& tracked_object,
    bool latch) override;

  void publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const override;
  void configCb(CVPublisherConfig& config, uint32_t level);

private:
  CVCodec codec;

  std::unique_ptr<dynamic_reconfigure::Server<CVPublisherConfig>> reconfigureServer;
  CVPublisherConfig config {CVPublisherConfig::__getDefault__()};
  std::vector<int32_t> cvOpts;
};

void Publisher::advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
  const image_transport::SubscriberStatusCallback& user_connect_cb,
  const image_transport::SubscriberStatusCallback& user_disconnect_cb, const ros::VoidPtr& tracked_object, bool latch)
{
  SimplePublisherPlugin<sensor_msgs::CompressedImage>::advertiseImpl(
    nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

  this->reconfigureServer = std::make_unique<dynamic_reconfigure::Server<CVPublisherConfig>>(this->nh());
  const auto f = std::bind(&Publisher::configCb, this, std::placeholders::_1, std::placeholders::_2);
  this->reconfigureServer->setCallback(f);
}

void Publisher::publish(const sensor_msgs::Image& message, const PublishFn& publish_fn) const
{
  if (!this->config.enable)
    return;

  this->codec.encode(message, this->config.format, this->cvOpts)
    .or_else([](const std::string& error) { ROS_ERROR("Failed to encode image using CV codec: %s", error.c_str()); })
    .map(publish_fn);
}

void Publisher::configCb(CVPublisherConfig& config, const uint32_t level)
{
  this->config = config;
  this->cvOpts = configToCvOptions(config);
}

}

PLUGINLIB_EXPORT_CLASS(cv_image_transport::Publisher, image_transport::PublisherPlugin)
