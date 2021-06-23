# Lab Final--- FastSLAM Localization
Lab Final ROS package of the course *Special Topics in Mobile Robots &amp; Self-Driving Cars*

## How to use it.

1. You should first download this file and unzip to you `~/catkin_ws/src/`
[fastslam_localization package download](https://drive.google.com/file/d/1mQ_SJIgDzpvG-4Qj21cR_9BoZZEySAj6/view)

2. git clone this repository.

3. catkin make under `~/catkin_ws` path.

4.
    roslaunch lab_final lab_final.launch
    
5. Open **another terminal**, manually publich to topic `/start`

```ros
rostic pub /start std_msgs/Int64 "data: 0"
```

## [Demo Video](https://drive.google.com/file/d/10-aPJ2C8aImhS_vXkZ2p_Gd_hSXDOeJX/view?usp=sharing)

## Design Flow:

1. c值計算  
2. R 參考作業  
3. M粒子數 先增大再縮小 發現最低可以達到 25顆粒子  
但為了求穩定 還是選擇 M=100  

4. 路徑規劃部分: 直接寫死目標點    
總共12個目標點，用定位控制做 control  
5. 快到下一個點的時候就換目標，動態目標點概念  
讓robot可以保持再一定的速度 不會歸零再重新啟動  
6. necking的設計 在8次較窄的地方 設較保守的控制量  