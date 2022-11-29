# MiramarAVDataAnalytics


## Prerequisites

##### Assumes that ROS Noetic is already installed on the system. For installation refer to - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

## Steps to Setup -

#### 1. **Download the github repo -**
```
git clone https://github.com/us-ignite/MiramarAVDataAnalytics.git
```
#### 2. Extract the zip folder and change the current directory 
```
cd MiramarAVDataAnalytics/ 
```
#### 3. Enter the following commands to setup ROS in the particular directory - 
```
catkin_make
source devel/setup.bash
```

## A. Steps to Generate a 3D Point Cloud Map from Lidar Data
##### Install the `ros_numpy` package if not already installed by typing the following command - 
```
sudo apt-get install ros-$release-ros-numpy
```
##### 1. Start the ROSMaster using the command in one terminal window - 
```
roscore
```
##### 2. In another terminal window play the rosbag from which the user wants to generate the 3D map - 
```
rosbag play <rosbag-name>.bag
```
For more information on playing rosbags & its parameters, refer to - [Rosbag Play](http://wiki.ros.org/rosbag/Commandline#play)
##### 3. In third terminal window type the following command to run the ROS node that generates the map and ensure that `duration` parameter is given -
```
rosrun build_map p2p_manual.py _duration:=<duration>
```
**Ex:**

`rosrun build_map p2p_manual.py _duration:=50`

`_duration:` parameter defines the total interval of the rosbag for which the user wants to generate the map. The code would generate the 3D map combining only the lidar point cloud for the defined interval.
###### Note - If the code throws `rostopic errors` then please check that the rostopic names rosbag is publishing matches with the rostopic names defined in the code. 

## B. Generating map using GPS data with key info visualization
##### 1. Start the ROSMaster using the command in one terminal window - 
```
roscore
```
##### 2. In another terminal window play the rosbag from which the user wants to generate the 3D map - 
```
rosbag play <rosbag-name>.bag
```
For more information on playing rosbags & its parameters, refer to - [Rosbag Play](http://wiki.ros.org/rosbag/Commandline#play)
##### 3. In third terminal window type the following command to run the ROS node that generates the map and ensure that `duration` & `with_mode` parameter is given -
```
rosrun gps_map_with_OA plot_map.py _duration:=<duration> _with_mode:=<'T' or 'F'>
```
**Ex:**

`rosrun gps_map_with_OA plot_map.py _duration:=50 _with_mode:='T'`

`rosrun gps_map_with_OA plot_map.py _duration:=50`

`rosrun gps_map_with_OA plot_map.py _duration:=50 _with_mode:='f'`
