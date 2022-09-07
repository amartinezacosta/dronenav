# Waypoint actions
uint8 WAYPOINT_ACTION_NONE = 1
uint8 WAYPOINT_ACTION_IMAGE = 2
uint8 WAYPOINT_ACTION_VIDEO = 4
uint8 WAYPOINT_ACTION_POINTCLOUD = 8
uint8 WAYPOINT_ACTION_QRCODE = 16

# Waypoint flags
uint8 PATH_LAST_FLAG = 1
uint8 FLUSH_QUEUE = 2

uint8 action
uint8 flags
geometry_msgs/Point position
float64 yaw
