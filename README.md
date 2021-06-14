# final
先執行main.cpp，在執行control.py(sudo python3 control.py /dev/ttyUSB0)
在main.cpp內95-111都是在用xbee接收資=訊息的部分然後用RPC function可以分類四個步驟，我分別傳L，就能用xbee收到/doLine/run 1，到doLine的RPC function，
接著再用thread到line_mode去執行跟著線的對應動作，那接著我在輸入B，xbee收到/doLine/run 0，先關掉Line的thread，在收到/doBlock/run 1，
去執行doBlock的RPC funciton，接著也有一個thread到block_mode去執行繞過障礙物的對應動作，以此類推，接著還有Apriltag、parking的部分。
那openmv那邊就是將final.py複製到main.py，再傳資料過去之前，他會先收到一個字，來判斷是要傳Line還是Apriltag的資料，如果沒收到那個字就部會傳任何鏡頭拍到的資訊過去。
