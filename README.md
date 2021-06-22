# final
執行main.cpp，不需要使用control.py
在main.cpp內96-429行是四個不同subtask，用一個變數去記錄現在在第幾個subtask，分別做跟著直線、繞過障礙物、垂直apriltag、停車。
每個subtask後都會用xbee傳一個字母，可以打開screen看(sudo screen /dev/ttyUSB0)，跟著直線(L)、繞過障礙物(B)、垂直apriltag(A)、停車(P)。
L:不斷的傳訊息叫openmv傳資訊回來，在根據傳回來的數值判斷車子要往哪裡走，在車子距離障礙物特定距離(用ping測量)，xbee會傳L出去，紀錄subtask的變數就會跳下一個數字，去執行下一個subtask。
B:以繞圓圈的方式繞過障礙物，到一個可以看到apriltag的位置，xbee會傳B出去，紀錄subtask的變數會跳下一個數字，執行下一個subtask。
A:傳訊息較openmv傳從apriltag讀到的資訊回來，根據角度與位置去判斷要如何走到垂直於apriltag的位置，等車子到垂直位置時，會用ping去測量車子與apriltag的距離，先存著，xbee會傳A出去，紀錄subtask的變數會跳下一個數字，執行下一個subtask。
P:原本會先有apriltag與停車位的d1、d2距離，在A的部分已先存著車子與apriltag的距離，所以將apriltag與停車位的距離d2相減，就能得到車子與停車位的d1、d2，車子根據這些數據判斷如何移動，到達目的地後xbee會傳P。
那openmv那邊就是將final.py複製到main.py，再傳資料過去之前，他會先收到一個字，來判斷是要傳Line還是Apriltag的資料，如果沒收到那個字就不會傳任何鏡頭拍到的資訊過去。
