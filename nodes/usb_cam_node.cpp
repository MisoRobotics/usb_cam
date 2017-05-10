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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");

  ros::NodeHandle node("~");

  std::string video_device_name;
  node.param("video_device", video_device_name, std::string("/dev/video0"));

  std::string config_namespace;
  node.param("config_namespace", config_namespace, std::string(""));

  usb_cam::UsbCamConfig config(node, config_namespace);

  usb_cam::UsbCamNodePublisher publisher(node, video_device_name, config, "", "image_raw");
  publisher.spin(true);

  printf("[usb_cam] usb_cam node %d finished; returning EXIT_SUCCESS\n", getpid());

  return EXIT_SUCCESS;
}
