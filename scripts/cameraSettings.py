import sys
from yaml import load
from usb_cam.srv import ChangeCameraSettings, ChangeCameraSettingsRequest
from usb_cam.msg import CameraSettings
import rospy

if len(sys.argv) < 2:
    print("Usage: %s SETTINGSSERVICE [SETTINGSFILE]" % sys.argv[0])
    print("  SETTINGSSERVICE is the usb_cam/ChangeCameraSettings service address to change camera settings.")
    print("  SETTINGSFILE is the file name of a YAML file containing 'names' and 'values' from a usb_cam/CameraSettings message, or name-value pairs for settings.  If not specified, settings will only be retrieved and printed rather than set.")
    exit(1)

req = ChangeCameraSettingsRequest()
if len(sys.argv) > 2:
    f = open(sys.argv[2], 'r')
    yaml = load(f)
    f.close()

    if 'names' in yaml and 'values' in yaml:
        req.names = yaml['names']
        req.values = yaml['values']
    else:
        for name in yaml:
            req.names.append(name)
            req.values.append(yaml[name])

rospy.init_node('camera_settings_script')
svc_address = sys.argv[1]
rospy.loginfo("Waiting for camera settings service at %s", svc_address)
rospy.wait_for_service(svc_address)
svc = rospy.ServiceProxy(svc_address, ChangeCameraSettings)
rospy.loginfo("%s %d settings at %s", "Changing" if len(sys.argv) > 2 else "Querying", len(req.names), svc_address)
resp = svc(req)

summary = 'Current settings:'
for i in range(len(resp.new_settings.names)):
    vtype = resp.new_settings.types[i]
    if vtype == CameraSettings.TYPE_INTEGER:
        value = str(resp.new_settings.values[i])
        vrange = "[%d, %d]" % (resp.new_settings.min_values[i], resp.new_settings.max_values[i])
    elif vtype == CameraSettings.TYPE_BOOLEAN:
        value = "True" if resp.new_settings.values[i] == 1 else "False"
        vrange = ""
    elif vtype == CameraSettings.TYPE_MENU or vtype == CameraSettings.TYPE_INTEGER_MENU:
        value = resp.new_settings.menu_items[resp.new_settings.menu_items_offsets[i] + resp.new_settings.values[i]]
        vrange = "{"
        first_item = True
        for j in range(resp.new_settings.min_values[i], resp.new_settings.max_values[i]+1):
            if first_item:
                first_item = False
            else:
                vrange += ", "
            vrange += resp.new_settings.menu_items[resp.new_settings.menu_items_offsets[i] + j]
        vrange += "}"
    else:
        value = "<Unknown type %d>" % vtype
        vrange = ""
    summary += "\n%s = %s %s" % (resp.new_settings.names[i], value, vrange)
rospy.loginfo(summary)
