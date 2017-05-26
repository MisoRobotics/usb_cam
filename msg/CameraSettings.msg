# Describes settings for a camera.
# All fields except menu_items are parts of an implicit dictionary where the names field contains the keys and other fields contain aspects of the values.
# This means all fields except menu_items will be the same size when published by a UsbCamNodePublisher.
# When setting camera settings by publishing to a topic with this message type, only names and values must be populated; all others will be ignored.

# Exact name of each setting
string[] names

# Value of each setting.
# If the setting is a boolean, this value will be 0 or 1.
# If the setting is a menu, this value will be the option described by menu_items[menu_items_offsets[i] + values[i]] where i is the corresponding index in values.
int32[] values

# Type of each setting; see constants below.
uint32[] types

uint32 TYPE_INTEGER = 1
uint32 TYPE_BOOLEAN = 2
uint32 TYPE_MENU = 3
uint32 TYPE_INTEGER_MENU = 9

# Default value of each setting.
int32[] default_values

# Minimum value of each setting; values below this value are invalid.
int32[] min_values

# Maximum value of each setting; values above this value are invalid.
int32[] max_values

# For menu settings only, contains the index in menu_items where the options for this setting begin, starting at min_values[i]
uint32[] menu_items_offsets

# Descriptions of menu options for all menu settings.
# This field is the only one that does not have the same size as the others.
string[] menu_items


