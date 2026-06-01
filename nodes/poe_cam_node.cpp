/*********************************************************************
 *
 * PoE camera node: mirrors usb_cam_node but uses rtsp protocol and OpenCV's 
 * VideoCapture instead of V4L2 to capture images from PoE cameras. This 
 * allows support for a wider range of cameras, including those that do not 
 * have Linux drivers.
 *
 *********************************************************************/
#include <filesystem>

#include <ros/ros.h>
#include <usb_cam/poe_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <memory>
#include <sstream>
#include <vector>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <misocpp/diagnostic_updater_wrapper.h>

namespace usb_cam {

namespace
{
std::string resolve_device_path(const std::string& p)
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
}  // namespace

class PoECamNode
{
  misocpp::DiagnosticHeartbeat heartbeat_;
  std::unique_ptr<misocpp::DiagnosticFrequency> diag_freq_image_raw_{ nullptr };
  std::unique_ptr<misocpp::DiagnosticFrequency> diag_freq_camera_info_{ nullptr };
  double expected_freq_;

public:
  ros::NodeHandle node_;
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;

  std::string camera_url_, camera_name_, camera_info_url_;
  int framerate_;
  std::string io_method_name_, pixel_format_name_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

  PoECam cam_;

  ros::ServiceServer service_start_, service_stop_, service_auto_reset_exposure_, reset_exposure_;

  PoECamNode() : node_("~")
  {
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);

    node_.param("video_url", camera_url_, std::string("rtsp://127.0.0.1:554/stream"));
    node_.param("pixel_format", pixel_format_name_, std::string("rgb24"));

    node_.param("camera_frame_id", img_.header.frame_id, std::string("poe_camera"));
    node_.param("camera_name", camera_name_, std::string("poe_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));

    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

    // start camera using PoECam API
    PoECam::pixel_format pf = PoECam::pixel_format_from_string(pixel_format_name_);

    if (!cam_.start(camera_url_, pf)) {
      ROS_ERROR("PoE camera failed to start: %s", camera_url_.c_str());
    }

    framerate_ = cam_.FPS();
    if (framerate_ <= 0) {
      framerate_ = 30;
      ROS_WARN("Invalid camera frame rate reported, defaulting to %d Hz", framerate_);
    }

    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(camera_url_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = cam_.Width();
      camera_info.height = cam_.Height();
      cinfo_->setCameraInfo(camera_info);
    }

    expected_freq_ = static_cast<double>(framerate_);
    std::filesystem::path topic = ros::this_node::getNamespace();
    topic /= image_pub_.getTopic();
    diag_freq_image_raw_ = std::make_unique<misocpp::DiagnosticFrequency>(topic.c_str(), expected_freq_, expected_freq_);

    std::string s(topic);
    s = s.erase(s.rfind('/'), std::string::npos);
    topic = s;
    topic /= "camera_info";
    diag_freq_camera_info_ = std::make_unique<misocpp::DiagnosticFrequency>(topic.c_str(), expected_freq_, expected_freq_);

    // services
    service_start_ = node_.advertiseService("start_capture", &PoECamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &PoECamNode::service_stop_cap, this);
  }

  virtual ~PoECamNode()
  {
    cam_.shutdown();
  }

  bool service_start_cap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    cam_.start_capturing();
    return true;
  }

  bool service_stop_cap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    cam_.stop_capturing();
    return true;
  }

  bool take_and_send_image()
  {
    if (!cam_.grab_image(&img_))
    {
      ROS_WARN_THROTTLE(5.0, "PoE camera did not respond, will retry.");
      return false;
    }

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    image_pub_.publish(img_, *ci);
    diag_freq_camera_info_->tick();
    diag_freq_image_raw_->tick();
    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (cam_.is_capturing() && !cam_.is_changing_config())
      {
        if (!take_and_send_image()) ROS_WARN("PoE camera did not respond in time.");
      }
      heartbeat_.update();
      loop_rate.sleep();
    }
    return true;
  }
};

}  // namespace usb_cam

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poe_cam");
  ros::AsyncSpinner spinner{2};
  spinner.start();
  usb_cam::PoECamNode n;
  n.spin();
  return EXIT_SUCCESS;
}
