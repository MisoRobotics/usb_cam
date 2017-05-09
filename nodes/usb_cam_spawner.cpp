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

// \todo =========Move to utils library project==============

std::string waitStatusString(int status) {
  std::stringstream ss;
  bool hasContent = false;
  if (WIFEXITED(status)) {
    int exitStatus = WEXITSTATUS(status);
    ss << "Exited(";
    if (exitStatus == EXIT_SUCCESS) ss << "SUCCESS";
    else if (exitStatus == EXIT_FAILURE) ss << "FAILURE";
    else ss << status;
    ss << ")";
    hasContent = true;
  }
  if (WIFSIGNALED(status)) {
    if (hasContent) ss << " ";
    int sig = WTERMSIG(status);
    ss << "Signaled(";
    if (sig == SIGINT) ss << "INT";
    else if (sig == SIGTERM) ss << "TERM";
    else if (sig == SIGKILL) ss << "KILL";
    else if (sig == SIGQUIT) ss << "QUIT";
    else ss << sig;
    ss << ")";
    hasContent = true;
  }
  if (WCOREDUMP(status)) {
    if (hasContent) ss << " ";
    ss << "CoreDump";
    hasContent = true;
  }
  if (WIFSTOPPED(status)) {
    if (hasContent) ss << " ";
    int sig = WSTOPSIG(status);
    ss << "StoppedBySignal(";
    if (sig == SIGINT) ss << "INT";
    else if (sig == SIGTERM) ss << "TERM";
    else if (sig == SIGKILL) ss << "KILL";
    else if (sig == SIGQUIT) ss << "QUIT";
    else ss << sig;
    ss << ")";
    hasContent = true;
  }
  if (WIFCONTINUED(status)) {
    if (hasContent) ss << " ";
    ss << "ResumedSigCont";
  }
  return ss.str();
}

// ===========================================================

pid_t spawnNode(const std::string& node_path, const std::string& device_path, const usb_cam::UsbCamConfig& config) {
  pid_t pid = fork();

  if (pid == 0)
  {
    // child process
    ROS_INFO("This is the child process");
    std::vector<std::string> args;

    args.push_back("rosrun");
    args.push_back("usb_cam");
    args.push_back("usb_cam_node");
    args.push_back("/usb_cam:=" + node_path);
    config.addAsRosrunArgs(args);
    args.push_back("_video_device:=" + device_path);

    ROS_INFO("Constructing exec info string");
    std::stringstream cmd;
    cmd << "rosrun";
    const char* charargs[args.size() + 1];
    int i = 0;
    for (auto const &arg : args) {
      charargs[i++] = arg.c_str();
      cmd << " " << arg;
    }
    charargs[i] = 0;

    ROS_INFO("Execing %s", cmd.str().c_str());
    int result = execvp("rosrun", (char **)charargs);
    ROS_ERROR("Continued past execv point! result = %d, errno = %s", result, strerror(errno));
    exit(EXIT_FAILURE);
  }
  else if (pid > 0)
  {
    // parent process
    return pid;
  }
  else
  {
    // fork failed
    ROS_ERROR("Unable to fork process to start camera node '%s'", node_path.c_str());
    return 0;
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam_spawner");
  ros::NodeHandle node("~");

  std::vector<usb_cam::UsbCamInfo> cameras = usb_cam::getCameras();

  std::map<std::string, std::string> roles;
  XmlRpc::XmlRpcValue camrolesnode;
  if (!node.getParam("/roles/cameras", camrolesnode) || camrolesnode.size() == 0) {
    ROS_ERROR("No camera roles found for current install in parameter server; expected parameters /roles/cameras/{ROLE} = {CAMERA}.");
    return EXIT_FAILURE;
  } else {
    for (auto& rolenode : camrolesnode) {
      std::string camera = (std::string)rolenode.second;
      roles.emplace(camera, rolenode.first);
    }
  }

  std::vector<pid_t> spawned_processes;
  for (auto const& camera : cameras) {
    if (roles.count(camera.serial_number) == 0) {
      ROS_WARN("No role found for USB camera at %s with serial %s", camera.devnode_path.c_str(), camera.serial_number.c_str());
      continue;
    }
    const std::string& role = roles[camera.serial_number];

    ROS_INFO("Checking %s", camera.devnode_path.c_str());
    std::string cam_node_base_path = "/cameras/" + camera.serial_number;
    XmlRpc::XmlRpcValue camnode;
    if (!node.getParam(cam_node_base_path, camnode)) {
      ROS_ERROR("Camera at %s with serial %s is not documented in the parameter server.",
             camera.devnode_path.c_str(), camera.serial_number.c_str());
      continue;
    }

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

    usb_cam::UsbCamConfig config = usb_cam::UsbCamConfig(node, cam_node_base_path + "/");
    std::string node_name = "/cameras/" + role;
    ROS_INFO("Spawning node at %s to serve images from USB camera at %s with serial %s: %s",
           node_name.c_str(), camera.devnode_path.c_str(), camera.serial_number.c_str(), config.toString().c_str());
    const pid_t pid = usb_cam::spawnNode(node_name, camera.devnode_path, config);
    spawned_processes.push_back(pid);
    //auto publisher = std::make_shared<usb_cam::UsbCamNodePublisher>(node, camera.devnode_path, config, camera.serial_number + "/", "image");
    //auto t = std::make_shared<std::thread>([=]() { publisher->spin(false); });
    //camthreads.push_back(t);
  }

  ROS_INFO("Started %d USB camera capture threads for usb_cams_node", (int)spawned_processes.size());

  ros::spin();

  const std::string TAG = "usb_cam_spawner";

  printf("[%s] Spawner node shut down; terminating %d child camera nodes\n", TAG.c_str(), (int)spawned_processes.size());
  for (const pid_t& pid : spawned_processes) {
    int status;
    int result = waitpid(pid, &status, WNOHANG);
    if (result == 0) {
      printf("[%s]   Killing child camera node %d\n", TAG.c_str(), pid);
      if (kill(pid, SIGINT) != 0) {
        fprintf(stderr, "Error killing child camera node %d: %s\n", pid, strerror(errno));
      }
    } else if (result == -1) {
      fprintf(stderr, "Error checking child camera node %d: %s\n", pid, strerror(errno));
    } else {
      printf("[%s]   Child camera node %d already terminated\n", TAG.c_str(), pid);
    }
  }

  printf("[%s] Waiting for child nodes to complete\n", TAG.c_str());
  for (const pid_t& pid : spawned_processes) {
    printf("[%s]   Waiting for %d to complete...\n", TAG.c_str(), pid);
    int status;
    waitpid(pid, &status, 0);
    printf("[%s]   Camera node %d completed, status %s\n", TAG.c_str(), pid, usb_cam::waitStatusString(status).c_str());
  }

  printf("[%s] Spawner node shutdown complete\n", TAG.c_str());

  return EXIT_SUCCESS;
}

