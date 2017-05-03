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

}



int main(int argc, char **argv)
{
  std::vector<usb_cam::UsbCamInfo> cameras = usb_cam::getCameras();

  ros::init(argc, argv, "usb_cameras");
  ros::NodeHandle node("~");

  std::vector<std::shared_ptr<std::thread>> camthreads;
  for (auto const& camera : cameras) {
    std::string cam_node_base_path = "/cameras/" + camera.serial_number;
    XmlRpc::XmlRpcValue camnode;
    if (!node.getParam(cam_node_base_path, camnode)) {
      ROS_ERROR("Camera at %s with serial %s is not documented in the parameter server.",
             camera.devnode_path.c_str(), camera.serial_number.c_str());
    } else {
      if (!camnode.hasMember("type")) {
        ROS_ERROR("USB V4L camera at %s with serial %s does not have a type documented in the parameter server.  Expected the parameter at /cameras/%s/type to be 'USB V4L'.",
               camera.devnode_path.c_str(), camera.serial_number.c_str(), camera.serial_number.c_str());
      } else {
        std::string camInfoType = (std::string)camnode["type"];
        if (camInfoType != "USB V4L") {
          ROS_ERROR("USB V4L camera at %s with serial %s is indicated as type '%s' in the parameter server rather than the expected type 'USB V4L'.",
                 camera.devnode_path.c_str(), camera.serial_number.c_str(), camInfoType.c_str());
        } else {
          usb_cam::UsbCamConfig config = usb_cam::UsbCamConfig(node, cam_node_base_path + "/");
          ROS_INFO("Serving images from USB camera at %s with serial %s: %s\n",
                 camera.devnode_path.c_str(), camera.serial_number.c_str(), config.toString().c_str());
          auto publisher = std::make_shared<usb_cam::UsbCamNodePublisher>(node, camera.devnode_path, config, camera.serial_number + "/", "image");
          auto t = std::make_shared<std::thread>([=]() { publisher->spin(false); });
          camthreads.push_back(t);
        }
      }
    }
  }

  ROS_DEBUG("Started %d USB camera capture threads for usb_cams_node", (int)camthreads.size());

  ros::spin();

  ROS_DEBUG("Waiting for USB camera capture threads to complete before exiting usb_cams_node");
  for (auto const& t : camthreads) {
    t->join();
  }

  return EXIT_SUCCESS;
}
