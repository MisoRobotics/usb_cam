/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <usb_cam/usb_cam_config.h>
#include <usb_cam/usb_cam_node_publisher.h>
#include <unistd.h>

#include <dynamic_reconfigure/server.h>
#include <usb_cam/V4LParametersConfig.h>

std::shared_ptr<usb_cam::UsbCamConfig> config;

void callback(usb_cam::V4LParametersConfig &dyn_config, uint32_t level) {
  ROS_INFO((std::string("usb_cam dynamic reconfigure:\n") +
           "brightness: %d # %d\n" +
           "contrast: %d # %d\n" +
           "saturation: %d # %d\n" +
           "sharpness: %d # %d\n" +
           "io_method: %s # %s\n" +
           "image_width: %d # %d\n" +
           "image_height: %d # %d\n" +
           "framerate: %s # %s\n" +
           "pixel_format: %s # %s\n" +
           "autofocus: %d # %d\n" +
           "focus: %d # %d\n" +
           "autoexposure: %d # %d\n" +
           "exposure: %d # %d\n" +
           "gain: %d # %d\n" +
           "auto_white_balance: %d # %d\n" +
           "white_balance: %d # %d\n" +
           "camera_frame_id: %s # %s\n" +
           "camera_name: %s # %s\n" +
           "camera_info_url: %s # %s\n").c_str(),
    dyn_config.brightness, config->brightness,
    dyn_config.contrast, config->contrast,
    dyn_config.saturation, config->saturation,
    dyn_config.sharpness, config->sharpness,
    dyn_config.io_method_name.c_str(), config->io_method_name.c_str(),
    dyn_config.image_width, config->image_width,
    dyn_config.image_height, config->image_height,
    std::to_string(dyn_config.framerate).c_str(), std::to_string(config->framerate).c_str(),
    dyn_config.pixel_format_name.c_str(), config->pixel_format_name.c_str(),
    dyn_config.autofocus, config->autofocus,
    dyn_config.focus, config->focus,
    dyn_config.autoexposure, config->autoexposure,
    dyn_config.exposure, config->exposure,
    dyn_config.gain, config->gain,
    dyn_config.auto_white_balance, config->auto_white_balance,
    dyn_config.white_balance, config->white_balance,
    dyn_config.camera_frame_id.c_str(), config->frame_id.c_str(),
    dyn_config.camera_name.c_str(), config->name.c_str(),
    dyn_config.camera_info_url.c_str(), config->info_url.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");

  ros::NodeHandle node("~");

  std::string video_device_name;
  node.param("video_device", video_device_name, std::string("/dev/video0"));

  std::string config_namespace;
  node.param("config_namespace", config_namespace, std::string(""));

  std::string image_path;
  node.param("image_path", image_path, std::string("image_raw"));

  config = std::make_shared<usb_cam::UsbCamConfig>(node, config_namespace);
ROS_INFO("usb_cam node has read config");
  usb_cam::UsbCamNodePublisher publisher(node, video_device_name, *config, "", image_path);

  std::cout << "PRINTING CONTROL VALUES\n";
  int value;
  for (auto const& ctrl : publisher.cam_.controls_) {
    if (publisher.cam_.getControlValue(ctrl.second, value)) {
      std::cout << "Value for " << ctrl.first << ": " << value << "\n";
    } else {
      std::cout << "Failed to get value for " << ctrl.first << "\n";
    }
  }
  std::cout << "PRINTED CONTROL VALUES\n";

  // Set up dynamic_reconfigure
  dynamic_reconfigure::Server<usb_cam::V4LParametersConfig> server;
  dynamic_reconfigure::Server<usb_cam::V4LParametersConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  publisher.spin(true);

  printf("[usb_cam] usb_cam node %d finished; returning EXIT_SUCCESS\n", getpid());

  return EXIT_SUCCESS;
}
