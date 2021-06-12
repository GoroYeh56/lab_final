# Lab Final--- FastSLAM Localization
Lab Final ROS package of the course *Special Topics in Mobile Robots &amp; Self-Driving Cars*

## How to use it.

1. git clone this repository.

2. catkin make under `~/catkin_ws` path.

3.
    roslaunch lab_final lab_final.launch
    
4. Open **another terminal**, manually publich to topic `/start`

```ros
rostic pub /start std_msgs/Int64 "data: 0"
```


### TODOs:

1. Tune the **Goal Locations** (X, Y, THETA)
2. 

