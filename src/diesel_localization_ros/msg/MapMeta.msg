# 地图版本元信息，transient_local 发布；下游据此判断地图是否过期/不匹配。

std_msgs/Header header

string version
geometry_msgs/Vector3 origin
float64 resolution
uint32 width
uint32 height
string pcd_sha256
string grid_sha256
string source
