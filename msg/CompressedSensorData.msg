#compressed sensor data
#センサーデータ圧縮用メッセージ（ただの構造体として使用）
#マップサイズ
#ピクセルサイズ
std_msgs/Int32 width
std_msgs/Int32 height
#クラスタデータ
std_msgs/Int32[] index #データ番号->マップ位置のインデックス
std_msgs/Int32[] size #マップセルに含まれるデータ数
geometry_msgs/Point[] pt #セル内データの平均値