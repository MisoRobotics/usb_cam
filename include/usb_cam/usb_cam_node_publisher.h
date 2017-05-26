#ifndef USB_CAM_NODE_PUBLISHER_H
#define USB_CAM_NODE_PUBLISHER_H

#include <string>
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <boost/shared_ptr.hpp>
#include <usb_cam/usb_cam.h>
#include <usb_cam/CameraSettings.h>
#include <usb_cam/ChangeCameraSettings.h>

namespace usb_cam {

// Advertises appropriate topics for the specified camera using the specified node
class UsbCamNodePublisher
{
  private:
    // private ROS node handle
    ros::NodeHandle node_;

    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;

    // parameters
    std::string video_device_name_;
    UsbCamConfig config_;
    //std::string start_service_name_, start_service_name_;
    bool streaming_status_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    ros::Publisher settings_publisher_;
    ros::Subscriber settings_subscriber_;
    ros::ServiceServer settings_service_;
    void onIncomingSettings(const ros::MessageEvent<usb_cam::CameraSettings const>& event);
    bool onChangeSettings(usb_cam::ChangeCameraSettings::Request& req, usb_cam::ChangeCameraSettings::Response& res);
    boost::shared_ptr<usb_cam::CameraSettings> makeSettingsMessage();

    ros::ServiceServer service_start_, service_stop_;
    bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    bool service_stop_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    bool take_and_send_image();

  public:
    UsbCamNodePublisher(ros::NodeHandle& node, const std::string& video_device_name, const UsbCamConfig& config, const std::string& camera_namespace, const std::string& image_path);

    virtual ~UsbCamNodePublisher();

    bool spin(bool spinRos = true);

    void adjustSettings(const std::vector<std::string>& names, const std::vector<int>& values);

    UsbCam cam_;
};


}

#endif
