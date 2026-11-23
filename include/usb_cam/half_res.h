#ifndef USB_CAM_HALF_RES_H
#define USB_CAM_HALF_RES_H

#include <cstring>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

namespace usb_cam {
/**
 * Scale an image the same way image_proc/resize does with use_scale:=true
 * (cv::resize(src, dst, cv::Size(0, 0), scale, scale, interpolation)), writing
 * straight into a sensor_msgs::Image. Header and encoding are copied from src.
 *
 * @return false for unsupported encodings or empty images.
 */
inline bool resizeImageMsg(const sensor_msgs::Image &src, double scale_x,
                           double scale_y, int interpolation,
                           sensor_msgs::Image &dst) {
  int cv_type;
  if (src.encoding == "rgb8" || src.encoding == "bgr8")
    cv_type = CV_8UC3;
  else if (src.encoding == "mono8")
    cv_type = CV_8UC1;
  else
    return false;
  if (src.width == 0 || src.height == 0 || src.data.empty())
    return false;

  const cv::Mat src_mat(static_cast<int>(src.height),
                        static_cast<int>(src.width), cv_type,
                        const_cast<uint8_t *>(src.data.data()), src.step);
  // Same output size cv::resize computes for dsize = Size(0, 0).
  const int width = cv::saturate_cast<int>(src.width * scale_x);
  const int height = cv::saturate_cast<int>(src.height * scale_y);
  if (width <= 0 || height <= 0)
    return false;

  dst.header = src.header;
  dst.encoding = src.encoding;
  dst.is_bigendian = src.is_bigendian;
  dst.width = static_cast<uint32_t>(width);
  dst.height = static_cast<uint32_t>(height);
  dst.step = static_cast<uint32_t>(width * src_mat.elemSize());
  dst.data.resize(static_cast<size_t>(dst.step) * height);

  // Resize directly into the message buffer (no extra copy).
  cv::Mat dst_mat(height, width, cv_type, dst.data.data(), dst.step);
  try {
    cv::resize(src_mat, dst_mat, cv::Size(0, 0), scale_x, scale_y,
               interpolation);
  } catch (const cv::Exception &) {
    return false;
  }
  if (dst_mat.data != dst.data.data()) {
    // OpenCV reallocated (should not happen): copy the result into the message.
    for (int row = 0; row < height; ++row)
      std::memcpy(&dst.data[row * dst.step], dst_mat.ptr(row), dst.step);
  }
  return true;
}

/** Scale camera_info the same way image_proc/resize does. */
inline sensor_msgs::CameraInfo
scaleCameraInfo(const sensor_msgs::CameraInfo &info, double scale_x,
                double scale_y) {
  sensor_msgs::CameraInfo out = info;
  out.height = cv::saturate_cast<int>(info.height * scale_y);
  out.width = cv::saturate_cast<int>(info.width * scale_x);
  out.roi.x_offset = static_cast<int>(info.roi.x_offset * scale_x);
  out.roi.y_offset = static_cast<int>(info.roi.y_offset * scale_y);
  out.roi.width = static_cast<int>(info.roi.width * scale_x);
  out.roi.height = static_cast<int>(info.roi.height * scale_y);
  out.K[0] = info.K[0] * scale_x; // fx
  out.K[2] = info.K[2] * scale_x; // cx
  out.K[4] = info.K[4] * scale_y; // fy
  out.K[5] = info.K[5] * scale_y; // cy
  out.P[0] = info.P[0] * scale_x; // fx
  out.P[2] = info.P[2] * scale_x; // cx
  out.P[3] = info.P[3] * scale_x; // T
  out.P[5] = info.P[5] * scale_y; // fy
  out.P[6] = info.P[6] * scale_y; // cy
  return out;
}

} // namespace usb_cam

#endif // USB_CAM_HALF_RES_H
