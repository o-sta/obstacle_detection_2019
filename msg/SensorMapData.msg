#sensor map data
#
#ROSヘッダ
std_msgs/Header header
#マップサイズ
#実測サイズ
std_msgs/Float32 width#
std_msgs/Float32 height#
std_msgs/Float32 res#
#ピクセルサイズ
std_msgs/Int32 widthInt
std_msgs/Int32 heightInt
#マップの中心点（センサ座標）
geometry_msgs/Point cp
#センサデータ
std_msgs/Int32[] index#map to pt data
std_msgs/Int32[] size#size of point in map cell 
geometry_msgs/Point[] pt