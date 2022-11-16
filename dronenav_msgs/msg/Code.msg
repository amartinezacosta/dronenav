#QR code message 

#ID
int32 id

#3D world coordinates
geometry_msgs/Point position
geometry_msgs/Vector3[] corners
geometry_msgs/Vector3 normal

#Pixel coordinates
uint32[] u
uint32[] v

#Tracking data
uint32 hit_streak
uint32 time_since_update

#QR code type and data
string type
string data