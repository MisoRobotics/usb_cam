/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Benjamin Pelletier.
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
#include <usb_cam/usb_cam.h>
#include <usb_cam/usb_cam_node_publisher.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <libudev.h>
#include <vector>
#include <map>
#include <utility>
#include <thread>
#include <unistd.h>
#include <ros/package.h>
#include <sstream>
#include <signal.h>
#include <sys/wait.h>
#include <sstream>

#include <errno.h>

#include "node_director/SpawnNode.h"

namespace usb_cam {

struct UsbCamInfo {
  std::string full_path;
  std::string devnode_path;
  std::string serial_number;
};

// Enumerate all V4L USB cameras along with their serial numbers
std::vector<UsbCamInfo> getCameras() {
  std::vector<UsbCamInfo> cameras;

  struct udev *udev;
  struct udev_device *dev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *list, *node;
  const char *path;

  udev = udev_new();
  if (!udev) {
    throw std::runtime_error("Cannot create udev to enumerate USB cameras");
  }

  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "video4linux");
  udev_enumerate_scan_devices(enumerate);

  list = udev_enumerate_get_list_entry(enumerate);
  udev_list_entry_foreach(node, list) {
    path = udev_list_entry_get_name(node);
    printf("Found path %s\n", path);
    dev = udev_device_new_from_syspath(udev, path);

    UsbCamInfo info = {};
    info.full_path = path;
    info.devnode_path = std::string(udev_device_get_devnode(dev));
    info.serial_number = std::string(udev_device_get_property_value(dev, "ID_SERIAL_SHORT"));
    //udev_device_get_property_value(dev, "ID_SERIAL"));
    //printf("SYSNAME=%s\n", udev_device_get_sysname(dev));
    cameras.push_back(info);

    udev_device_unref(dev);
  }

  return cameras;
}

bool spawnNode(ros::ServiceClient& client, const std::string& node_path, const std::string& device_path, const std::string& config_namespace) {

  std::vector<std::string> args;
  args.push_back("/usb_cam:=" + node_path);
  args.push_back("_video_device:=" + device_path);
  args.push_back("_config_namespace:=" + config_namespace);
  args.push_back("_image_path:=image");

  node_director::SpawnNode srv;
  srv.request.ros_package = "usb_cam";
  srv.request.executable_name = "usb_cam_node";
  srv.request.args = args;
  if (!client.call(srv))
    return false;
  return srv.response.id > 0;
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam_spawner");
  ros::NodeHandle node("~");

  ros::ServiceClient client = node.serviceClient<node_director::SpawnNode>("/node_director/spawn_node");

  std::vector<usb_cam::UsbCamInfo> cameras = usb_cam::getCameras();

  // Read camera roles from parameter server
  std::map<std::string, std::string> roles;
  XmlRpc::XmlRpcValue camrolesnode;
  if (!node.getParam("/install/roles/cameras", camrolesnode) || camrolesnode.size() == 0) {
    ROS_ERROR("No camera roles found for current install in parameter server; expected parameters /install/roles/cameras/{ROLE} = {CAMERAID}.");
    return EXIT_FAILURE;
  } else {
    for (auto& rolenode : camrolesnode) {
      std::string camera = (std::string)rolenode.second;
      roles.emplace(camera, rolenode.first);
    }
  }

  // Spawn a usb_cam node for each applicable camera
  for (auto const& camera : cameras) {
    if (roles.count(camera.serial_number) == 0) {
      ROS_WARN("No role found for USB camera at %s with serial %s", camera.devnode_path.c_str(), camera.serial_number.c_str());
      continue;
    }
    const std::string& role = roles[camera.serial_number];

    // Read the root camera configuration node from the parameter server
    ROS_INFO("Checking %s", camera.devnode_path.c_str());
    std::string cam_node_base_path = "/cameras/" + camera.serial_number;
    XmlRpc::XmlRpcValue camnode;
    if (!node.getParam(cam_node_base_path, camnode)) {
      ROS_ERROR("Camera at %s with serial %s is not documented in the parameter server.",
             camera.devnode_path.c_str(), camera.serial_number.c_str());
      continue;
    }

    // Make sure this camera is a USB V4L camera
    if (!camnode.hasMember("type")) {
      ROS_ERROR("USB V4L camera at %s with serial %s does not have a type documented in the parameter server.  Expected the parameter at /cameras/%s/type to be 'USB V4L'.",
             camera.devnode_path.c_str(), camera.serial_number.c_str(), camera.serial_number.c_str());
      continue;
    }
    std::string camInfoType = (std::string)camnode["type"];
    if (camInfoType != "USB V4L") {
      ROS_ERROR("USB V4L camera at %s with serial %s is indicated as type '%s' in the parameter server rather than the expected type 'USB V4L'.",
             camera.devnode_path.c_str(), camera.serial_number.c_str(), camInfoType.c_str());
      continue;
    }

    // Set up and spawn node to serve this camera
    std::string node_name = "/cameras/" + role;
    ROS_INFO("Spawning node at %s to serve images from USB camera at %s with serial %s and configuration at %s",
           node_name.c_str(), camera.devnode_path.c_str(), camera.serial_number.c_str(), cam_node_base_path.c_str());
    bool success = usb_cam::spawnNode(client, node_name, camera.devnode_path, cam_node_base_path + "/");
    if (!success) {
      ROS_ERROR("Couldn't spawn node at %s", node_name.c_str());
    }
  }

  ROS_INFO("USB camera node spawning complete");

  return EXIT_SUCCESS;
}

