
# Zemo Rover

This is a two wheel differential drive robot. This package is implemented on real hardware. 



## Deployment

Arduino Mega2560 is used for low level control of encoder motors. Upload the arduino sketch from zemo_rover/src/Arduino/Sketch/Turtle_Bot_ROS. For building this sketch use library files PID,Filter and Motor.

Clone this repository on robot computer and also base station system. Connect both ROS environment by setting up the ROS_MASTER_URI and ROS_IP on both sides. 
On robot terminal run:

```bash
roslaunch zemo_rover zemo_robot.launch
```

On work station system run:

```bash
roslaunch zemo_rover zemo_work_station.launch
```


## Required ROS Packages

For zemo robot companion computer we need differential-drive and rosserial ROS packages.

For work station computer we need rviz, teleop_twist_keyboard.