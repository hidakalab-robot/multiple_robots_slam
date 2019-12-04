uint8 NORMAL = 0
uint8 COS = 1

std_msgs/Header header
string status # move_base, VFH
int32 calc_range_method # normal , cos
float64[] range_pattern # avoidance ranges pattern
string[] descriptions # ranges description
string scan_frame_id
float32 scan_angle_min
float32 scan_angle_max
float32 scan_angle_increment
