# map-building-ros  
The mapping and relocalization module of CityU's blind navigation system.
## 1. Prerequisites  
1.1. Ubuntu 18.04.  
1.2. ROS version Kinetic or Melodic fully installation  
1.3. Ceres Solver Follow Ceres Installation  
1.4. Sophus  
```
  git clone http://github.com/strasdat/Sophus.git
  git checkout a621ff
```
## 2. Features  
2.1 A successful reimplementation of visual-inertial framework for blind positioning.  
2.2 A noise-eliminated dense reconstruction method based on the Octree management and depth filtering.  
2.3 An introduction of 3D dense points into each keyframes for map saving and reloading once loop gets detected.  
2.4 Ground floor reprojection; multi-layer costmap for independent global path planning and local obstacle avoidance.  
2.5 A Rosbridge-based ROSBridgeClient at the Android end and a user-friendly Android user interface.  

## 3. How to run  
### 3.1 Launch the RealSense camera node  
Running the following command at the termial to launch the camera nodes:
```
roslaunch realsense2_camera rs_camera.launch
```
  
In order to enable IMU (Inertial Motion Unit) and get aligned depth stream, please modify the RealSense launch file as follow:
```
<arg name="enable_gyro"    default="true"/>
<arg name="enable_accel"    default="true"/>

<arg name="enable_sync"    default="true"/>
<arg name="align_depth"    default="true"/>

<arg name="unite_imu_method"    default="linear_interpolation"/>
```
  
If you need dense point cloud for local obstacle avoidance, please enable the point cloud publisher:
```
<arg name="enable_pointcloud"    default="true"/>
```
  
### 3.2 Initialize the ROS workspace for the mapping and positioning algorithms  
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
git clone https://github.com/lucienyoung/map-building-ros.git
catkin_make
```
  
### 3.3 Run the mapping and positioning algorithm  
The global mapping launch files have been integrated into one, you can only run the following commands:
```
roslaunch vins_estimator realsense_color.launch
```
  
For local point cloud downsampling, please run the following:
```
rosrun rs_pcl_filter rs_downsample
```
Note that you should record the navigation goal (destination) along with map building for any future retrieval. Clicking the record button at your Android human-machine interface allows you enter the recording mode, and you can input specified goal names into the text bar for recording and receive their real-time position and orientation from ROS for updating.
  
### 3.4 Real-time path planning
For real-time path planning, you can refer to the following repository:
```
pass
```
  
The relavant detail can be found at ros-navigation WiKi.

### 3.5 Path planning simulation
If you'd like to simulate and test the path planing performance without real-time running, please refer to the [**rbx1**](https://github.com/pirobot/rbx1.git) simulation framework. To install, you can use the following commands:
```
sudo apt-get install ros-<rosdistro>-turtlebot-*
sudo apt-get install ros-<rosdistro>-arbotix-*

cd ~/catkin_ws/src
git clone https://github.com/pirobot/rbx1.git
cd ..
catkin_make
```

In order to enter the simulation environment, you can input the following commands into your terminal；
```
roslaunch rbx1_bringup fake_turtlebot.launch

roslaunch rbx1_nav fake_amcl.launch map:=test_map.yaml

rosrun rviz rviz -d `rospack find rbx1_nav`/nav.rviz
```
Where the test_map.yaml definds the meta-parameter of your pre-build global map. You can utilize command to store your map once mapping procedure gets finished.
```
rosrun map_server map_saver map:/projected_map -f /home/your_user_name/test_map
```
