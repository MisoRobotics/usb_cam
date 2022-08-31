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
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <memory>
#include <sstream>
#include <std_srvs/Empty.h>
#include <thread>
#include <usb_cam/device_utils.h>

namespace usb_cam {

//! \brief Manual Mode on V4L2 auto_exposure setting
const int AUTO_EXPOSURE_MANUAL_MODE = 1;

//! \brief Aperture Priority Mode on V4L2 auto_exposure setting
const int AUTO_EXPOSURE_APERTURE_PRIORITY_MODE = 3;

//! \brief Delay time in seconds to wait before set auto_exposure setting
const int WAIT_CHANGING_AUTO_EXPOSURE_SEC = 2;

class UsbCamNode
{
  // ROS diagnostic updater object.
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::Heartbeat heartbeat_;
  std::unique_ptr<diagnostic_updater::FrequencyStatusParam> frequency_status_param_;
  std::unique_ptr<diagnostic_updater::FrequencyStatus> frequency_status_;
  double expected_freq_;

public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
  std::string serial_number_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, bits_per_pixel_, exposure_, brightness_, contrast_, saturation_,
      sharpness_, focus_, white_balance_, gain_, power_line_frequency_, gamma_, backlight_compensation_;
  bool autofocus_, autoexposure_, auto_white_balance_, reset_exposure_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    // grab the parameters
    node_.param("serial_no", serial_number_, std::string(""));
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    node_.param("bits_per_pixel", bits_per_pixel_, 12);
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("reset_exposure", reset_exposure_, false);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);
    node_.param("power_line_frequency", power_line_frequency_, 1);
    node_.param("gamma", gamma_, 50);
    node_.param("backlight_compensation", backlight_compensation_, 1);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    if (!serial_number_.empty())
    {
      str_map map_dev_serial = get_serial_dev_info();
      clear_unsupported_devices(map_dev_serial, pixel_format_name_);

      bool found = false;
      auto it = map_dev_serial.cbegin();
      for (; it != map_dev_serial.cend(); ++it)
      {
        std::size_t found_pos = serial_number_.find(it->second);
        if (found_pos != std::string::npos)
        {
          found = true;
          video_device_name_ = it->first;
          break;
        }
      }

      if (!found)
      {
        ROS_FATAL("USB camera with serial number '%s' cannot be found.", serial_number_.c_str());
        node_.shutdown();
        return;
      }
    }

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s %d bpp) at %i FPS", camera_name_.c_str(),
             video_device_name_.c_str(), image_width_, image_height_, io_method_name_.c_str(),
             pixel_format_name_.c_str(), bits_per_pixel_, framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, bits_per_pixel_, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    if (power_line_frequency_ >= 0)
    {
      cam_.set_v4l_parameter("power_line_frequency", power_line_frequency_);
    }

    if (gamma_ >= 0)
    {
      cam_.set_v4l_parameter("gamma", gamma_);
    }

    if (backlight_compensation_ >= 0)
    {
      cam_.set_v4l_parameter("backlight_compensation", backlight_compensation_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check reset exposure
    if (reset_exposure_ && !autoexposure_)
    {
      // Executing the reset exposure routine automatically. Some cameras are
      // over exposed using the exposure_auto as manual_mode directly (without
      // setting to aperture priority mode before).
      cam_.set_v4l_parameter(
        "exposure_auto",
        AUTO_EXPOSURE_APERTURE_PRIORITY_MODE
      );
      std::this_thread::sleep_for(
        std::chrono::seconds{ WAIT_CHANGING_AUTO_EXPOSURE_SEC }
      );
      cam_.set_v4l_parameter("exposure_auto", AUTO_EXPOSURE_MANUAL_MODE);
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }
    else
    {
      // Just loading the file configuration without reset exposure routine.
      if (!autoexposure_)
      {
        // turn off exposure control
        cam_.set_v4l_parameter("exposure_auto", AUTO_EXPOSURE_MANUAL_MODE);
        // change the exposure level
        cam_.set_v4l_parameter("exposure_absolute", exposure_);
      }
      else
      {
        // turn on exposure auto control
        cam_.set_v4l_parameter(
          "exposure_auto",
          AUTO_EXPOSURE_APERTURE_PRIORITY_MODE
        );
      }
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }

    // Set hardware ID for diagnostic updater.
    diagnostic_updater_.setHardwareID("none");

    // Heartbeat
    diagnostic_updater_.add(heartbeat_);

    // Frequency Status
    expected_freq_ = static_cast<double>(framerate_);
    frequency_status_param_ = std::make_unique<diagnostic_updater::FrequencyStatusParam>(&expected_freq_, &expected_freq_);
    ROS_ASSERT(frequency_status_param_);
    frequency_status_ = std::make_unique<diagnostic_updater::FrequencyStatus>(*frequency_status_param_, "FrequencyStatus");
    ROS_ASSERT(frequency_status_);
    diagnostic_updater_.add(*frequency_status_);
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    if(!cam_.grab_image(&img_)) ros::shutdown();
    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    // publish the image
    image_pub_.publish(img_, *ci);
    frequency_status_->tick();

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      diagnostic_updater_.update();
      loop_rate.sleep();

    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
