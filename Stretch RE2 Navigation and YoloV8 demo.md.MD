# Stretch RE2 Navigation and YoloV8 demonstration


This readme provides a proper way to demonstrate the pin point navigation and real time YoloV8 prediction using ROS 1. The requirement to run this demonstration is that user has to have static map of the environment recorded using mapping algorithm provided by default ROS navigation stack. Please refer to the appendix to execute the mapping script and create a custom map. The sensors use in this demonstaration are: 1.LiDAR, 2. RGBD camera, 3. Differential driver with odometry control feedback. For further information please refer to the https://docs.hello-robot.com/0.2/ 

## Setup
In order to do untethered operation, the user is recommended to install NoMachine VNC viewer or any open sourece VNC application. In this demonstration, it required to be connected to same LAN wireless connection and the default KU Secure access point is used. Then, if the connection is secured, open NoMachine VNC viewer apps in the user PC, and add nx://10.10.165.219:4000 as the default IP for the Stretch RE2. Please notice that this is dynamic IP which may change depends on the access point connected by the host. For SSH connection, the user may use ssh://10.10.165.219:22. However, as the ID and password for login to the Stretch RE2 is private, please inqure the team to obtain it.  

### Navigation 
First, initialize the home poistion of the robot's arm and gripper. 
```
$  stretch_robot_home.py
```
To run the navigation, make sure to put map yaml fine name in the argument (in this example, it's in ${HELLO_FLEET_PATH}/maps/VRIlabNEW.yaml).
It will execute the navigation 2D (x,y) stack using the default Dijkstra path planning algorithm and DWA algorithm for obstacle avoidance. For localization, the script uses AMCL for differential driver AGV.
```
$  roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/VRIlabNEW.yaml 
```

The Rviz windows will pop out and the user may add additional window inside the Rviz to show the real time camera stream for example. 


### YoloV8 Demonstration

The YoloV8 demonstrates the model to track the stream image from RGBD camera and classify the images for the leaf detection.

To run the YoloV8, first the user is required to run the RGBD camera driver.

```
$  roslaunch stretch_core d435i_low_resolution.launch
```

[TO:DO] adding the rotate camera script to the launch file thus it will run simultaneously with the RGBD camera driver.

At the current version, the camera is rotated 90 degrees clockwise by default installation. Thus, a help script to return it back to normal is required to be executed.

```
$  cd
ipython rotate_rs_image.py
```

Finally, to run the YoloV8, first make sure the model for the detection in the parameter is correctly selected. Then, run the yolov8 launch file. 

```
$  roslaunch ultralytics_ros tracker.launch
```
To change the model for the YoloV8 track change directory to the Yolov8 package and modify the model name tag in the launch file.

```
$  cd /home/hello-robot/catkin_ws/src/ultralytics_ros/launch
$  gedit tracker.launch
```