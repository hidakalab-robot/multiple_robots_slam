std_msgs/Header header
bool overlap
float64 resolution #元マップの解像度
int32[] order #マップの順番
geometry_msgs/Point[] size #各マップのサイズ(x,y)
geometry_msgs/Point[] origin #各マップの原点(x,y)
geometry_msgs/Point[] rect_tl #被っている部分の左上座標(x,y)
geometry_msgs/Point[] rect_br #被っている部分の右下座標(x,y)
geometry_msgs/Point[] tl #被っている部分の左上座標(x,y)
geometry_msgs/Point[] br #被っている部分の右下座標(x,y)