#QR code message 

#ID
int32 id

#3D world coordinates
geometry_msgs/Point position
geometry_msgs/Vector3[] corners

#Pixel coordinates
uint32[] u
uint32[] v

#QR code type and data
string type
string data