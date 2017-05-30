usb_cam [![Build Status](https://api.travis-ci.org/bosch-ros-pkg/usb_cam.png)](https://travis-ci.org/bosch-ros-pkg/usb_cam)
=======

#### A ROS Driver for V4L USB Cameras
This package is based off of V4L devices specifically instead of just UVC.

For full documentation, see [the ROS wiki](http://ros.org/wiki/usb_cam).

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.

### Miso upgrades

The interface with the camera itself is encapsulated in the UsbCam class.  The features a camera adds to a node are encapsulated in UsbCamNodePublisher (multiple of these publishers could be applied to a single node).  An executable node that hosts and publishes one camera is contained in usb_cam_node.

#### usb_cam_node

Default node name is usb_cam.

##### Parameters
* **video_device**: The path to the usb_camera to be hosted (e.g., /dev/video0)

* **config_namespace**: The namespace in which to look for camera settings parameters (defaults to "", implying that private node parameters may be used to configure the camera)

* **image_path**: The name of the subtopic in which to publish image-related topics.  Defaults to image_raw, which means image-related topics will be subtopics of /usb_cam/image_raw

There are also a number of parameters that may be specified to set camera settings upon startup; see the UsbCamConfig constructor in usb_cam_config.cpp for a complete list and exact names.

##### Topics
In the paths below, ~ denotes the node namespace, which defaults to /usb_cam

* **~/image_raw (sensor_msgs/Image)**: The raw image topic, as well as host of many subtopics populated by image_transport.

* **~/camera_info (sensor_msgs/CameraInfo)**: Information about the intrinsic characteristics of the camera, managed by camera_info_manager.

* **~/settings (usb_cam/CameraSettings)**: Dynamic camera settings values following the dynamic-parameters-via-topic pattern.  Settings are checked and published upon startup, and whenever a change request is issued.  If a different node publishes a message to this topic, this node will ignore all fields of that message except names and values which it will use to attempt to change those settings.  The names field may contain any subset of the available settings, including no entries (which would result in no settings being changed, and the node republishing new current settings to ~/settings).

To manually change a setting via the current_settings topic on the command line, type something like:

```rostopic pub /usb_cam/settings usb_cam/CameraSettings "{names:['Exposure, Auto', 'Exposure (Absolute)'], values:[1, 100]}"```

Settings are changed in the order they are specified.

##### Services
* **~/change_settings (usb_cam/ChangeCameraSettings)**: Attempt to change the specified camera settings, and then return the new settings of the camera after the changes.  The settings will be changed in the order they are specified in the service request.

#### usb_cam_spawner

To automatically spawn a node (via node_director) for each USB camera attached to a computer, just start this node.  If there are parameters in the form of:

```/install/roles/cameras/{ROLE} = {SERIAL_NUMBER}```

then usb_cam_spawner will set each node's namespace to /cameras/ROLE.  If these parameters are not present, each node's namespace will be set to /cameras/SERIAL_NUMBER.

All the camera settings parameters available to usb_cam_node may also be specified on a per-camera basis when spawning nodes via usb_cam_spawner; they must be located in the corresponding node's namespace.

Recommended usage is to create a .yaml file containing the intrinsic information for all cameras that may be connected to the system.  It would look like this:

```
'B461A840':
  camera_name: 'Logitech C615'
  type: 'USB V4L'
  camera_info_url: 'file://$(find miso_config)/equipment/cameras/B461A840/camera_info.yaml'
  image_width: 1920
  image_height: 1080
  pixel_format: 'mjpeg'
  io_method: 'mmap'
'BCE5EBA0':
  camera_name: 'Logitech C310 (gray)'
  type: 'USB V4L'
  camera_info_url: 'file://$(find miso_config)/equipment/cameras/BCE5EBA0/camera_info.yaml'
  image_width: 1280
  image_height: 960
  pixel_format: 'mjpeg'
  io_method: 'mmap'```

