#データが存在する場合のみ、index、size、ptに要素を追加する。そのため、要素数は同じになる。
int32[] index               #マップのセル番号(= row * width/resolution + col)
int32[] size                #セル内の点数
geometry_msgs/Point[] pt    #セルの重心