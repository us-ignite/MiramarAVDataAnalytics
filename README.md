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

`_duration:` parameter defines the total interval of the rosbag for which the user wants to generate the map. The code would generate the 3D map combining only the lidar point cloud for the defined interval.

**Ex:**

`rosrun build_map p2p_manual.py _duration:=50`

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

`_duration:` parameter defines the total interval of the rosbag for which the user wants to generate the map.

`_with_mode:` parameter defines that the route will be displayed in color-coded form based on the vehicle mode of the vehicle.

**Ex:**

`rosrun gps_map_with_OA plot_map.py _duration:=50 _with_mode:='T'`

`rosrun gps_map_with_OA plot_map.py _duration:=50`

`rosrun gps_map_with_OA plot_map.py _duration:=50 _with_mode:='f'`
## C. Calculating Operational Analytics from a particular trip using the rosbag data
#### Prerequisite - 

The rosbag is contained in the current working directory from where the Rosnode for calculating the Operational Analytics is being called.
##### 1. Start the ROSMaster using the command in one terminal window - 
```
roscore
```
##### 2. In another terminal window, run the following command that calculates & displays the Operational Analytics parameters for a particular trip using the rosbag data based on the defined time interval.
```
rosrun reg operational_analytics.py _name:='<rosbag-name>.bag' _start:=<start-time> _interval:=<end-duration> _create:=<'T' or 'F'> _vel_info:=<'T' or 'F'> _mode_info:=<'T' or 'F'> _trans_info:=<'T' or 'F'> _topics:=<topic-names>
```

`_name:` parameter is used to represent the name of the rosbag located in the `CWD` directory for which the operational analytics if to be done.

`_start:` parameter repreents the start duration of the time from when the analytics parameters are to be calculated. For ex, `0` value means that analytics will be performed from the beginning or a value of `10` would mean that analysis starts from 10 secs after the rosbag start time.

`_interval:` parameter represents the stopping time till where the analytics need to be performed. A value of `50` would mean that the analysis would be performed for 50 secs after the `_start` parameter time. If the time exceeds then analysis is performed till the end of rosbag is reached.

`_create:` parameter is used to run a subsegment of program that converts the data of the topics into csv files which helps in faster analysis calculations for subsequent runs on the same data.

`_vel_info:` parameter decides to calculate the Operational Analytics information from the velocity topic.

`_mode_info:` parameter decides to calculate the Operational Analytics information from the vehicle-mode topic.

`_trans_info:` parameter decides to calculate the Operational Analytics information from the transmission-mode topic.

`_topics:` parameter is used to define the topic names that are to be converted into csv files so that Operational Analytics can be performed on those files.

**Ex:**

`rosrun reg operational_analytics.py _name:='sample.bag' _start:=0 _interval:=50 _create:='T' _vel_info:='T' _mode_info:='T' _trans_info:='T' _topics:="['/actuation','/gps','/imu','/tf','/transmission','/vehiclemode','/velocity']"`
