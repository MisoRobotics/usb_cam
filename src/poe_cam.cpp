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
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/poe_cam.h>
#include <vector>

namespace usb_cam {

PoECam::PoECam()
  : is_capturing_(false), is_changing_config_(false), monochrome_(false),
    pixel_format_(CV_8UC3), framerate_(30) {
}

PoECam::~PoECam() {
  shutdown();
}

bool PoECam::start(const std::string& camera_url, pixel_format pf) {
  camera_url_ = camera_url;

  // Determine if monochrome based on pixel format
  switch (pf) {
    case PIXEL_FORMAT_YUVMONO10:
    case PIXEL_FORMAT_GREY:
      monochrome_ = true;
      pixel_format_ = CV_8UC1;
      break;
    case PIXEL_FORMAT_RGB24:
    case PIXEL_FORMAT_YUYV:
    case PIXEL_FORMAT_UYVY:
    case PIXEL_FORMAT_MJPEG:
    default:
      monochrome_ = false;
      pixel_format_ = CV_8UC3;
      break;
  }

  // Force FFMpeg to use TCP
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);

  // Initialize video capture with PoE camera
  video_capture_ = std::make_unique<cv::VideoCapture>();
  
  // Try to open the camera
  bool opened = false;

  ROS_INFO("Opening camera with URL: %s", camera_url.c_str());
  opened = video_capture_->open(camera_url, cv::CAP_FFMPEG);

  if (!opened) {
    ROS_ERROR("Failed to open camera: %s", camera_url.c_str());
    return false;
  }

  // Set codec
  video_capture_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('h', 'v', 'c', '1'));

  // Get camera properties from device
  framerate_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FPS));
  camera_width_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FRAME_WIDTH));
  camera_height_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FRAME_HEIGHT));

  ROS_INFO("Capture params: %d x %d : %d fps", camera_width_, camera_height_, framerate_);

  // Allocate image buffer
  image_ = std::make_unique<camera_image_t>();
  image_->width = camera_width_;
  image_->height = camera_height_;
  image_->bytes_per_pixel = monochrome_ ? 1 : 3;
  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = (char *)calloc(image_->image_size, sizeof(char));
  
  if (!image_->image) {
    ROS_ERROR("Failed to allocate image buffer");
    return false;
  }

  memset(image_->image, 0, image_->image_size * sizeof(char));
  
  start_capturing();
  return true;
}

bool PoECam::open_camera() {
  if (camera_url_.empty()) {
    ROS_ERROR("No camera URL configured for PoECam");
    return false;
  }

  // Force FFMpeg to use TCP
  setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);

  video_capture_ = std::make_unique<cv::VideoCapture>();
  ROS_INFO("Opening camera with URL: %s", camera_url_.c_str());

  if (!video_capture_->open(camera_url_, cv::CAP_FFMPEG)) {
    ROS_ERROR("Failed to open camera: %s", camera_url_.c_str());
    return false;
  }

  video_capture_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('h', 'v', 'c', '1'));
  return true;
}

bool PoECam::reopen_camera() {
  if (!video_capture_) {
    return open_camera();
  }

  ROS_WARN("Reopening PoE camera stream: %s", camera_url_.c_str());
  video_capture_->release();
  return open_camera();
}

void PoECam::shutdown(void) {
  stop_capturing();
  
  if (video_capture_ && video_capture_->isOpened()) {
    video_capture_->release();
  }
  video_capture_.reset();

  if (image_ && image_->image) {
    free(image_->image);
    image_->image = nullptr;
  }
  image_.reset();
}

bool PoECam::grab_image(sensor_msgs::Image* msg) {
  if (!grab_image())
    return false;

  msg->header.stamp = ros::Time::now();
  
  if (monochrome_) {
    fillImage(*msg, "mono8", image_->height, image_->width,
              image_->width, image_->image);
  } else {
    fillImage(*msg, "rgb8", image_->height, image_->width,
              3 * image_->width, image_->image);
  }

  return true;
}

bool PoECam::grab_image() {
  if (!video_capture_ || !video_capture_->isOpened() || !is_capturing_) {
    return false;
  }

  if (!video_capture_->read(current_frame_)) {
    ROS_WARN("Failed to read frame from camera, attempting to reopen stream");
    if (!reopen_camera()) {
      ROS_ERROR("Failed to reopen camera stream");
      return false;
    }
    if (!video_capture_->read(current_frame_)) {
      ROS_ERROR("Failed to read frame from camera after reopen");
      return false;
    }
  }

  if (current_frame_.empty()) {
    ROS_ERROR("Empty frame received");
    return false;
  }

  process_image(current_frame_);
  image_->is_new = 1;
  return true;
}

void PoECam::process_image(const cv::Mat& frame) {
  if (!image_ || !image_->image) {
    return;
  }

  cv::Mat target_frame = frame.clone();

  // Convert to appropriate format
  if (monochrome_) {
    if (target_frame.channels() != 1) {
      cv::cvtColor(target_frame, target_frame, cv::COLOR_BGR2GRAY);
    }
  } else {
    if (target_frame.channels() != 3) {
      cv::cvtColor(target_frame, target_frame, cv::COLOR_YUV2BGR_I420);
    }
    // OpenCV uses BGR, but we need RGB for ROS
    cv::cvtColor(target_frame, target_frame, cv::COLOR_BGR2RGB);
  }

  // Resize if needed
  if (target_frame.rows != image_->height || target_frame.cols != image_->width) {
    cv::resize(target_frame, target_frame, cv::Size(image_->width, image_->height));
  }

  // Copy frame data to image buffer
  if (monochrome_) {
    memcpy(image_->image, target_frame.data, image_->image_size);
  } else {
    // Ensure continuous memory layout
    if (target_frame.isContinuous()) {
      memcpy(image_->image, target_frame.data, image_->image_size);
    } else {
      char *dest = image_->image;
      for (int i = 0; i < target_frame.rows; ++i) {
        memcpy(dest, target_frame.ptr<uchar>(i), 
               target_frame.cols * target_frame.channels());
        dest += target_frame.cols * target_frame.channels();
      }
    }
  }
}

void PoECam::set_auto_focus(int value) {
  if (video_capture_ && video_capture_->isOpened()) {
    video_capture_->set(cv::CAP_PROP_AUTOFOCUS, value);
  }
}

bool PoECam::device_supports_pixel_format(const std::string& device, 
                                          const std::string& pixel_format) {
  // PoE cameras support most formats through video_stream_opencv
  // This is a simplified check
  return true;
}

PoECam::pixel_format PoECam::pixel_format_from_string(const std::string& str) {
  if (str == "yuyv")
    return PIXEL_FORMAT_YUYV;
  else if (str == "uyvy")
    return PIXEL_FORMAT_UYVY;
  else if (str == "mjpeg")
    return PIXEL_FORMAT_MJPEG;
  else if (str == "yuvmono10")
    return PIXEL_FORMAT_YUVMONO10;
  else if (str == "rgb24")
    return PIXEL_FORMAT_RGB24;
  else if (str == "grey")
    return PIXEL_FORMAT_GREY;
  else
    return PIXEL_FORMAT_UNKNOWN;
}

void PoECam::stop_capturing(void) {
  if (!is_capturing_)
    return;

  is_capturing_ = false;
  if (video_capture_ && video_capture_->isOpened()) {
    video_capture_->release();
  }
}

void PoECam::start_capturing(void) {
  if (is_capturing_)
    return;

  if (video_capture_ && video_capture_->isOpened()) {
    is_capturing_ = true;
  }
}

bool PoECam::is_capturing() {
  return is_capturing_;
}

bool PoECam::is_changing_config() {
  return is_changing_config_;
}

void PoECam::is_changing_config(bool is_changing) {
  is_changing_config_ = is_changing;
}

}  // namespace usb_cam
