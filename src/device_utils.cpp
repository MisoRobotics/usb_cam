#include <boost/algorithm/string.hpp>
#include <libudev.h>
#include <string>
#include <usb_cam/device_utils.h>
#include <usb_cam/usb_cam.h>
#include <ros/ros.h>


str_map get_serial_dev_info()
{
  str_map devices;

  struct udev *udev = nullptr;
  struct udev_device *dev = nullptr;
  struct udev_device *parent_dev = nullptr;
  struct udev_enumerate *enumerate = nullptr;
  struct udev_list_entry *list, *node = nullptr;
  const char *path = nullptr;

  udev = udev_new();
  if (udev)
  {
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);

    list = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(node, list)
    {
      path = udev_list_entry_get_name(node);
      dev = udev_device_new_from_syspath(udev, path);

      std::string dev_file_name_path = std::string(udev_device_get_devnode(dev));

      parent_dev = udev_device_get_parent_with_subsystem_devtype(
        dev,
        "usb",
        "usb_device");

      if (parent_dev != nullptr)
      {
        const auto serial = udev_device_get_sysattr_value(parent_dev, "serial");
        if (serial) {
          std::string dev_serial = std::string(serial);
          devices[dev_file_name_path] = dev_serial;
        }
        else {
          ROS_WARN_STREAM("Found device with null serial: '" << dev_file_name_path << "'");
        }
      }

      udev_device_unref(dev);
    }

    udev_unref(udev);
  }

  return devices;
}

std::string get_pixel_format_v4l(str_map& maps, std::string pixel_format)
{
  std::string result = pixel_format;
  str_map::iterator it = maps.find(pixel_format);
  if (it != maps.end())
  {
    result = it->second;
  }
  return result;
}

str_map get_pixel_format_map()
{
  str_map pixel_format_map;
  // V4L uses MJPG and usb_cam uses MJPEG
  pixel_format_map["MJPEG"] = "MJPG";
  return pixel_format_map;
}

void clear_unsupported_devices(str_map& maps, std::string pixel_format)
{
  str_map pixel_format_map = get_pixel_format_map();
  std::string upper_str = get_pixel_format_v4l(pixel_format_map,
    boost::to_upper_copy<std::string>(pixel_format));

  auto it = maps.cbegin();
  while (it != maps.cend())
  {
    if (usb_cam::UsbCam::device_supports_pixel_format(it->first, upper_str))
    {
      ++it;
    }
    else
    {
      it = maps.erase(it);
    }
  }
}
