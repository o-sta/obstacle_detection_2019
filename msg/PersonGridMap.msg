std_msgs/Header header #ROSヘッダ
#グリッドマップに関する情報
float32 width       #幅[m]
float32 height      #高さ[m]
float32 resolution  #解像度[m/cell]
#データレイヤー
obstacle_detection_2019/PersonGridMapData[] layer
