# sara_description

## Dependencies
- velodyne gazebo plugin & velodyne description
   ``` 
   sudo apt-get install ros-melodic-velodyne* 
   ```
- ouster description
   ``` 
   cd ~/catkin_ws/src 
   git clone https://github.com/wilselby/ouster_example.git 
   ```
- realsense2
   ``` 
   sudo apt-get install ros-melodic-realsense2-description 
   ```
- husky and/or jackal and/or gvrbot
   ```
   git clone https://github.com/husky/husky.git 
   git clone https://github.com/westpoint-robotics/usma_jackal.git 
   git clone https://github.com/westpoint-robotics/usma_gvrbot.git 
   ```

## The SARA Sensors Package

Install the SARA Sensors package in the same catkin workspace as the above dependencies (the ones not installed with sudo apt-get)

```
cd ~/catkin_ws/src
git clone https://github.com/westpoint-robotics/sara_description.git
```
Within the sara_description package, there are versions of husky.urdf.xacro, jackal.urdf.xacro, and gvrbot.urdf.xacro that are modified to allow for the incorporation of the SARA sensors modular frame (a modular frame constructed with 8020 for mounting a set of sensors in a particular configuration for SLAM, stereo vision, etc.)  Once you have the husky, jackal, and gvrbot packages installed in your catkin workspace, you will have to go to the urdf directory within each package, and replace that robot's primary urdf with the one from the SARA package.  Having done that, you will be able to run the demo launch file for any of the three robots as follows:

1. ` roslaunch sara_description sara_sensors_demos.launch which_robot:=husky `
2. ` roslaunch sara_description sara_sensors_demos.launch which_robot:=jackal `
3. ` roslaunch sara_description sara_sensors_demos.launch which_robot:=gvrbot `

The "which_robot" parameter allows you to indirectly launch a husky, jackal, or gvrbot gazebo simulation.  It also is used to determine whether to use the wider or narrower version of the 8020 frame (sensor placement within the frame is always the same), and what the origin of the frame is relative to the robot in question.  For the GVR-Bot, the frame is currently centered, instead of at the front of the robot as in the case of the husky and jackal.  This may be modified in line 45 of the sara.urdf.xacro

```
 <xacro:if value="${version == 'gvrbot'}">
   <origin xyz="0.0 0.0 0.1675" rpy="0 0 0" />
 </xacro:if> 
```
 
When you are running the sara_sensors_demos luanch files, you can then also run RViz using
```
roslaunch <current robot>_viz view_robot.launch
```

You can also use RViz alone with:
```
roslaunch <current robot>_viz view_model.launch
```

### TODO:
```
 -Make more self-contained to eliminate need to edit robot repositories
 -Make OS1-64 work in Gazebo without using Velodyne
 -Add instructions for working with actual robots ```
