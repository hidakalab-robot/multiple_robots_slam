std_msgs/Header header
bool overlap
float64 resolution #元マップの解像度
int32[] order #マップの順番
geometry_msgs/Point[] size #マップのサイズ
geometry_msgs/Point[] rect_tl #被っている部分の左上座標
geometry_msgs/Point[] rect_br #被っている部分の右下座標
geometry_msgs/Point[] tl #被っている部分の左上座標
geometry_msgs/Point[] br #被っている部分の右下座標