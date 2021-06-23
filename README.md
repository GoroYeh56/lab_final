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

