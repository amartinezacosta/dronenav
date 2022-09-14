#Constants

#Flying 
string TAKEOFF_POSITIONING_STATE = TAKEOFF_POSITIONING
string TAKEOFF_YAWING_STATE = TAKEOFF_YAWING
string HOVERING_STATE = HOVERING 
string POSITIONING_STATE = POSITIONING
string REACHED_STATE = REACHED 

#Landed
string LANDED_TOUCHDOWN_STATE = LANDED_TOUCHDOWN
string LANDED_POSITIONING = LANDED_POSITIONING
string LANDED_YAWING = LANDED_YAWING

#Message status for dronenav
string state
uint32 waypoint_queue_size
Waypoint current_waypoint
geometry_msgs/Point current_position
geometry_msgs/Point target_position
float64 current_yaw
float64 target_yaw