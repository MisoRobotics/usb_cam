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

#include <filesystem>

#include <algorithm>
#include <cmath>
#include <mutex>

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <memory>
#include <sstream>
#include <vector>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <sensor_msgs/JointState.h>
#include <usb_cam/device_utils.h>
#include <misocpp/diagnostic_updater_wrapper.h>

namespace usb_cam {

namespace
{
// udev lists e.g. /dev/video0; flippy-config may use /dev/v4l/by-path/... symlinks to the same node.
std::string resolve_v4l_device_path(const std::string& p)
{
  try
  {
    return std::filesystem::weakly_canonical(p).string();
  }
  catch (const std::exception&)
  {
    return p;
  }
}

bool getOrderedJointPositions(const sensor_msgs::JointState& msg,
                              const std::vector<std::string>& joint_names,
                              std::vector<double>* positions)
{
  if (!positions)
  {
    return false;
  }
  positions->clear();
  positions->reserve(joint_names.size());
  for (const auto& joint_name : joint_names)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (it == msg.name.end())
    {
      return false;
    }
    const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
    if (idx >= msg.position.size())
    {
      return false;
    }
    positions->push_back(msg.position[idx]);
  }
  return true;
}

bool jointsNearTarget(const std::vector<double>& current,
                      const std::vector<double>& target,
                      double tolerance)
{
  if (current.size() != target.size())
  {
    return false;
  }
  for (size_t i = 0; i < current.size(); ++i)
  {
    if (std::abs(current[i] - target[i]) > tolerance)
    {
      return false;
    }
  }
  return true;
}

// /joint_states is merged arm-then-rail (joint_state_filter: r1 then s1).
const std::vector<std::string> kJointStatesOrder = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "slider_1"};

// /behaviors/points/* use /miso/joint_map order (slider_1, joint_1, ... joint_6).
bool reorderFromJointMapOrder(const std::vector<double>& joint_map_order,
                              const std::vector<std::string>& joint_map,
                              const std::vector<std::string>& output_order,
                              std::vector<double>* output)
{
  if (!output || joint_map_order.size() != joint_map.size())
  {
    return false;
  }

  output->clear();
  output->reserve(output_order.size());
  for (const auto& joint_name : output_order)
  {
    const auto it = std::find(joint_map.begin(), joint_map.end(), joint_name);
    if (it == joint_map.end())
    {
      return false;
    }
    const size_t idx = static_cast<size_t>(std::distance(joint_map.begin(), it));
    output->push_back(joint_map_order[idx]);
  }
  return true;
}
}  // namespace

//! \brief Manual Mode on V4L2 auto_exposure setting
const int AUTO_EXPOSURE_MANUAL_MODE = 1;

//! \brief Aperture Priority Mode on V4L2 auto_exposure setting
const int AUTO_EXPOSURE_APERTURE_PRIORITY_MODE = 3;

//! \brief Delay time in seconds to wait before set auto_exposure setting
const int WAIT_CHANGING_AUTO_EXPOSURE_SEC = 2;

//! \brief Timer period in seconds between calls to reset camera exposure setting
const double AUTO_RESET_EXPOSURE_PERIOD = 60.0;

//! \brief Timer period in seconds between calls to reset camera exposure setting
const double ONE_SHOT_RESET_EXPOSURE_WAIT = 1.0;

class UsbCamNode
{
  misocpp::DiagnosticHeartbeat heartbeat_;
  std::unique_ptr<misocpp::DiagnosticFrequency> diag_freq_image_raw_{ nullptr };
  std::unique_ptr<misocpp::DiagnosticFrequency> diag_freq_camera_info_{ nullptr };
  double expected_freq_, auto_reset_exposure_period_;
  ros::Timer auto_reset_exposure_timer_;
  bool enable_auto_reset_exposure_;

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
  bool autofocus_, autoexposure_, auto_white_balance_;
  bool init_cam_reset_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_, service_auto_reset_exposure_, reset_exposure_;
  ros::Subscriber joint_states_sub_;

  bool gate_capture_on_joint_state_;
  std::string joint_states_topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> fryer_inspection_joints_;
  double joint_position_tolerance_;
  std::mutex joint_state_mutex_;
  bool has_joint_state_;
  bool at_fryer_inspection_joints_;

  bool service_start_cap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    cam_.start_capturing();
    return true;
  }

  bool reset_exposure_call(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    ros::Rate srv_rate(5);
    bool timeout = 20.0;
    float start_time = ros::Time::now().toSec();
    while (node_.ok() && (ros::Time::now().toSec() - start_time < timeout))
    {
      if (init_cam_reset_)
      {
        break;
      }
      srv_rate.sleep();
    }
    resetExposureSettings();
    return true;
  }

  bool service_stop_cap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    cam_.stop_capturing();
    return true;
  }

  bool service_auto_reset_exposure( std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res )
  {
    enable_auto_reset_exposure_ = req.data;
    res.success = true;
    res.message = "";
    return true;
  }

  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (!gate_capture_on_joint_state_)
    {
      return;
    }

    std::vector<double> current_joints;
    if (!getOrderedJointPositions(*msg, joint_names_, &current_joints))
    {
      return;
    }

    const bool at_target = jointsNearTarget(
        current_joints, fryer_inspection_joints_, joint_position_tolerance_);

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      has_joint_state_ = true;
      at_fryer_inspection_joints_ = at_target;
    }
  }

  bool shouldCaptureFrame()
  {
    if (!gate_capture_on_joint_state_)
    {
      return true;
    }

    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return has_joint_state_ && at_fryer_inspection_joints_;
  }

  UsbCamNode() :
      node_("~"),
      gate_capture_on_joint_state_(false),
      joint_states_topic_("/joint_states"),
      joint_position_tolerance_(0.05),
      has_joint_state_(false),
      at_fryer_inspection_joints_(false)
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
    node_.param("auto_reset_exposure_period", auto_reset_exposure_period_, AUTO_RESET_EXPOSURE_PERIOD); // No reset if < 0
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

    node_.param("joint_states_topic", joint_states_topic_, joint_states_topic_);
    node_.param("joint_position_tolerance", joint_position_tolerance_, joint_position_tolerance_);
    node_.param("gate_capture_on_joint_state", gate_capture_on_joint_state_, gate_capture_on_joint_state_);

    const bool is_fryer_camera = camera_name_.compare(0, 9, "fryer_cam") == 0;
    if (is_fryer_camera || gate_capture_on_joint_state_)
    {
      ros::NodeHandle nh;
      std::vector<double> fryer_inspection_joint_map_order;
      if (!nh.getParam("/behaviors/points/fryer_inspection", fryer_inspection_joint_map_order) ||
          fryer_inspection_joint_map_order.empty())
      {
        ROS_WARN(
            "%s: /behaviors/points/fryer_inspection not available; capture is not gated on joint state.",
            camera_name_.c_str());
        gate_capture_on_joint_state_ = false;
      }
      else
      {
        std::vector<std::string> joint_map;
        if (!nh.getParam("/miso/joint_map", joint_map))
        {
          ROS_FATAL(
              "%s: joint gating requires /miso/joint_map to interpret /behaviors/points/fryer_inspection.",
              camera_name_.c_str());
          node_.shutdown();
          return;
        }

        if (joint_map.size() != fryer_inspection_joint_map_order.size())
        {
          ROS_FATAL(
              "%s: /miso/joint_map (%zu) and /behaviors/points/fryer_inspection (%zu) must have the same length.",
              camera_name_.c_str(), joint_map.size(), fryer_inspection_joint_map_order.size());
          node_.shutdown();
          return;
        }

        if (!reorderFromJointMapOrder(
                fryer_inspection_joint_map_order, joint_map, kJointStatesOrder, &fryer_inspection_joints_))
        {
          ROS_FATAL(
              "%s: failed to reorder fryer_inspection joints to /joint_states order (joint_1..joint_6, slider_1).",
              camera_name_.c_str());
          node_.shutdown();
          return;
        }

        joint_names_ = kJointStatesOrder;
        gate_capture_on_joint_state_ = true;

        joint_states_sub_ = node_.subscribe(
            joint_states_topic_, 1, &UsbCamNode::jointStatesCallback, this);
      }
    }
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    init_cam_reset_ = false;

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);
    reset_exposure_ = node_.advertiseService("manual_reset_exposure", &UsbCamNode::reset_exposure_call, this);
    service_auto_reset_exposure_ = node_.advertiseService("reset_exposure", &UsbCamNode::service_auto_reset_exposure, this);

    if (!serial_number_.empty())
    {
      str_map map_dev_serial = get_serial_dev_info();
      clear_unsupported_devices(map_dev_serial, pixel_format_name_);

      std::vector<std::string> serial_matches;
      for (const auto& dev_serial : map_dev_serial)
      {
        if (serial_number_ == dev_serial.second)
        {
          serial_matches.push_back(dev_serial.first);
        }
      }

      if (serial_matches.empty())
      {
        ROS_FATAL("USB camera with serial number '%s' cannot be found.", serial_number_.c_str());
        node_.shutdown();
        return;
      }

      if (serial_matches.size() == 1)
      {
        video_device_name_ = serial_matches.front();
      }
      else
      {
        // Several V4L nodes can report the same USB serial; keep the `video_device` param (e.g. by-path)
        // when it resolves to the same dev node as one of the udev entries.
        const std::string wanted_resolved = resolve_v4l_device_path(video_device_name_);
        bool device_ok = false;
        for (const auto& path : serial_matches)
        {
          if (resolve_v4l_device_path(path) == wanted_resolved)
          {
            device_ok = true;
            break;
          }
        }
        if (!device_ok)
        {
          std::ostringstream oss;
          oss << "USB serial '" << serial_number_ << "' matches several devices; set per-camera video_device "
                 "in world_launch (e.g. /dev/v4l/by-path/...). Candidates:";
          for (const auto& path : serial_matches)
          {
            oss << ' ' << path;
          }
          ROS_FATAL("%s", oss.str().c_str());
          node_.shutdown();
          return;
        }
      }
    }

    if (cinfo_->isCalibrated())
    {
      ROS_INFO(
          "%s: loaded camera intrinsics from %s",
          camera_name_.c_str(),
          camera_info_url_.empty() ? "camera_info_manager default" : camera_info_url_.c_str());
    }
    else
    {
      if (!camera_info_url_.empty())
      {
        ROS_WARN(
            "%s: failed to load camera intrinsics from '%s'; aruco_detect will warn about K matrix zeros.",
            camera_name_.c_str(), camera_info_url_.c_str());
      }
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
      cam_.set_v4l_parameter("white_balance_automatic", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_automatic", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // Just loading the file configuration without reset exposure routine.
    if (!autoexposure_)
    {
      // turn off exposure control
      cam_.set_v4l_parameter("auto_exposure", AUTO_EXPOSURE_MANUAL_MODE);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_time_absolute", exposure_);
    }
    else
    {
      // turn on exposure auto control
      cam_.set_v4l_parameter(
        "auto_exposure",
        AUTO_EXPOSURE_APERTURE_PRIORITY_MODE
      );
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

    std::string ns = ros::this_node::getNamespace();
    expected_freq_ = static_cast<double>(framerate_);
    std::filesystem::path topic = ns;
    topic /= image_pub_.getTopic();
    diag_freq_image_raw_ =
        std::make_unique<misocpp::DiagnosticFrequency>(topic.c_str(), expected_freq_, expected_freq_);
    ROS_ASSERT(diag_freq_image_raw_);
    std::string s(topic);
    s = s.erase(s.rfind('/'), std::string::npos);
    topic = s;
    topic /= "camera_info";
    diag_freq_camera_info_ =
        std::make_unique<misocpp::DiagnosticFrequency>(topic.c_str(), expected_freq_, expected_freq_);
    ROS_ASSERT(diag_freq_camera_info_);

    enable_auto_reset_exposure_ = true;
    bool timer_oneshot = auto_reset_exposure_period_ <= 0;
    double timer_period = timer_oneshot ? ONE_SHOT_RESET_EXPOSURE_WAIT : auto_reset_exposure_period_;
    auto_reset_exposure_timer_ = node_.createTimer(
      ros::Duration(timer_period),
      boost::bind(&UsbCamNode::checkAutoResetExposure, this, _1),
      timer_oneshot
    );
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
    diag_freq_camera_info_->tick();
    diag_freq_image_raw_->tick();

    return true;
  }

  void resetExposureSettings()
  {
    // Set auto exposure to on
    cam_.set_v4l_parameter(
      "auto_exposure",
      AUTO_EXPOSURE_APERTURE_PRIORITY_MODE
    );

    std::this_thread::sleep_for(
      std::chrono::seconds{ WAIT_CHANGING_AUTO_EXPOSURE_SEC }
    );

    // Set exposure time to off
    cam_.set_v4l_parameter("auto_exposure", AUTO_EXPOSURE_MANUAL_MODE);

    std::this_thread::sleep_for(
      std::chrono::seconds{ WAIT_CHANGING_AUTO_EXPOSURE_SEC }
    );

    // Set the manual exposure level
    cam_.set_v4l_parameter("exposure_time_absolute", exposure_);
  }

  void checkAutoResetExposure(const ros::TimerEvent&)
  {
    if (enable_auto_reset_exposure_)
    {
      resetExposureSettings();
    }
    init_cam_reset_ = true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      const bool ready_to_capture =
          cam_.is_capturing() && !cam_.is_changing_config();
      if (ready_to_capture && shouldCaptureFrame())
      {
        if (!take_and_send_image())
        {
          ROS_WARN("USB camera did not respond in time.");
        }
      }
      heartbeat_.update();
      loop_rate.sleep();
    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  // Will need more threads if we add more callbacks
  ros::AsyncSpinner spinner{ 2 };
  spinner.start();
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
