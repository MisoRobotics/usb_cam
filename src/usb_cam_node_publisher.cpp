/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Benjamin Pelletier
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <usb_cam/usb_cam_node_publisher.h>

namespace usb_cam {

bool UsbCamNodePublisher::service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  cam_.start_capturing();
  return true;
}

bool UsbCamNodePublisher::service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
{
  cam_.stop_capturing();
  return true;
}

UsbCamNodePublisher::UsbCamNodePublisher(ros::NodeHandle& node, const std::string& video_device_name, const UsbCamConfig& config, const std::string& camera_namespace, const std::string& image_path) :
  node_(node),
  video_device_name_(video_device_name),
  config_(config)
{
  // advertise the main image topic
  image_transport::ImageTransport it(node_);
  image_pub_ = it.advertiseCamera(camera_namespace + image_path, 1);

  img_.header.frame_id = config_.frame_id;
  cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, config_.name, config_.info_url));

  // create Services
  service_start_ = node_.advertiseService(camera_namespace + "start_capture", &UsbCamNodePublisher::service_start_cap, this);
  service_stop_ = node_.advertiseService(camera_namespace + "stop_capture", &UsbCamNodePublisher::service_stop_cap, this);

  // check for default camera info
  if (!cinfo_->isCalibrated())
  {
    cinfo_->setCameraName(video_device_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = img_.header.frame_id;
    camera_info.width = config_.image_width;
    camera_info.height = config_.image_height;
    cinfo_->setCameraInfo(camera_info);
  }

  ROS_INFO_STREAM("Starting streaming for " << config_.toString());

  if (!cam_.start(video_device_name_, config_))
  {
    node_.shutdown();
  }
}

UsbCamNodePublisher::~UsbCamNodePublisher()
{
  cam_.shutdown();
}

bool UsbCamNodePublisher::take_and_send_image()
{
  // grab the image
  cam_.grab_image(&img_);

  // grab the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
  ci->header.frame_id = img_.header.frame_id;
  ci->header.stamp = img_.header.stamp;

  // publish the image
  image_pub_.publish(img_, *ci);

  return true;
}

bool UsbCamNodePublisher::spin(bool spinRos)
{
  ros::Rate loop_rate(config_.framerate);
  while (node_.ok())
  {
    if (cam_.is_capturing()) {
      if (!take_and_send_image()) ROS_WARN("USB camera at %s did not respond in time.", video_device_name_.c_str());
    }
    if (spinRos)
      ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}

}
