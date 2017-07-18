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
#include <usb_cam/usb_cam_config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <array>
#include <vector>
#include <string>

#include <ros/node_handle.h>

namespace usb_cam {

UsbCamConfig::UsbCamConfig()
  : io_method_name(std::string("mmap")) // possible values: mmap, read, userptr
  , pixel_format_name("mjpeg")          // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
  , name(std::string("head_camera"))
  , info_url(std::string(""))
  , frame_id(std::string("head_camera"))
  , image_width(640)
  , image_height(480)
  , framerate(60)
  , exposure(100)
  , brightness(-1)                      // 0-255, -1 "leave alone"
  , contrast(-1)                        // 0-255, -1 "leave alone"
  , saturation(-1)                      // 0-255, -1 "leave alone"
  , sharpness(-1)                       // 0-255, -1 "leave alone"
  , focus(-1)                           // 0-255, -1 "leave alone"
  , white_balance(4000)
  , gain(-1)                            // 0-100?, -1 "leave alone"
  , autofocus(false)                    // enable/disable autofocus
  , autoexposure(false)                 // enable/disable autoexposure
  , auto_white_balance(true)            // enable/disable auto white balance temperature
  , autofocus_defined(false)
  , autoexposure_defined(false)
  , auto_white_balance_defined(false)
{ }

UsbCamConfig::UsbCamConfig(ros::NodeHandle& node, const std::string& param_namespace) : UsbCamConfig::UsbCamConfig() {
  // grab the parameters
  node.param(param_namespace + "brightness", brightness, brightness);
  node.param(param_namespace + "contrast", contrast, contrast);
  node.param(param_namespace + "saturation", saturation, saturation);
  node.param(param_namespace + "sharpness", sharpness, sharpness);
  node.param(param_namespace + "io_method", io_method_name, io_method_name);
  node.param(param_namespace + "image_width", image_width, image_width);
  node.param(param_namespace + "image_height", image_height, image_height);
  node.param(param_namespace + "framerate", framerate, framerate);
  node.param(param_namespace + "pixel_format", pixel_format_name, pixel_format_name);
  autofocus_defined = node.hasParam(param_namespace + "autofocus");
  node.param(param_namespace + "autofocus", autofocus, autofocus);
  node.param(param_namespace + "focus", focus, -1);
  autoexposure_defined = node.hasParam(param_namespace + "autoexposure");
  node.param(param_namespace + "autoexposure", autoexposure, autoexposure);
  node.param(param_namespace + "exposure", exposure, exposure);
  node.param(param_namespace + "gain", gain, gain);
  auto_white_balance_defined = node.hasParam(param_namespace + "auto_white_balance");
  node.param(param_namespace + "auto_white_balance", auto_white_balance, auto_white_balance);
  node.param(param_namespace + "white_balance", white_balance, white_balance);

  // load the camera info
  //TODO: None of these parameters should be prefaced with 'camera_' because all parameters in this namespace are assumed to be camera parameters.
  //Not changing for now to support backwards compatibility.
  node.param(param_namespace + "camera_frame_id", frame_id, frame_id);
  node.param(param_namespace + "camera_name", name, name);
  node.param(param_namespace + "camera_info_url", info_url, info_url);
}

std::string UsbCamConfig::toString() const {
  std::array<char, 512> buffer;
  int r = snprintf(buffer.data(), buffer.size(), "'%s' at %dx%d via %s (%s) at %i FPS", name.c_str(),
                 image_width, image_height, io_method_name.c_str(), pixel_format_name.c_str(), framerate);
  if (r < 0) {
    throw std::runtime_error("Could not format string in UsbCamConfig::toString");
  }
  return std::string(buffer.data(), buffer.size());
}

std::map<std::string, std::string> UsbCamConfig::toMap() const {
  std::map<std::string, std::string> map;
  map.emplace("brightness", std::to_string(brightness));
  map.emplace("contrast", std::to_string(contrast));
  map.emplace("saturation", std::to_string(saturation));
  map.emplace("sharpness", std::to_string(sharpness));
  map.emplace("io_method", io_method_name);
  map.emplace("image_width", std::to_string(image_width));
  map.emplace("image_height", std::to_string(image_height));
  map.emplace("framerate", std::to_string(framerate));
  map.emplace("pixel_format", pixel_format_name);
  map.emplace("autofocus", std::to_string(autofocus));
  map.emplace("focus", std::to_string(focus));
  map.emplace("autoexposure", std::to_string(autoexposure));
  map.emplace("exposure", std::to_string(exposure));
  map.emplace("gain", std::to_string(gain));
  map.emplace("auto_white_balance", std::to_string(auto_white_balance));
  map.emplace("white_balance", std::to_string(white_balance));

  map.emplace("camera_frame_id", frame_id);
  map.emplace("camera_name", name);
  map.emplace("camera_info_url", info_url);
  return map;
}

std::string UsbCamConfig::toYaml() const {
  auto map = toMap();
  std::stringstream ss;
  for (auto const &kvp : map) {
    ss << kvp.first << ": '" << kvp.second << "'\n";
  }
  return ss.str();
}

void UsbCamConfig::addAsRosrunArgs(std::vector<std::string>& args) const {
  auto map = toMap();
  for (auto const &kvp : map) {
    if (kvp.second.find(" ") == std::string::npos) {
      args.push_back("_" + kvp.first + ":=" + kvp.second);
    } else {
      args.push_back("_" + kvp.first + ":='" + kvp.second + "'");
    }
  }
}

}
