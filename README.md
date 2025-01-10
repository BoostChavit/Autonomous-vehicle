# Setup
## Requirement
- Jetson nano tx2
- Ubuntu 18.04
- Python 2.7
- ROS melodic (https://wiki.ros.org/melodic/Installation/Ubuntu)
- BLDC (https://github.com/vedderb/bldc-tool)

## ROS setup
1. mkdir -p wecar_ws/src
2. cd wecar_ws/src && catkin_init_workspace
3. cd ../ && catkin_make
4. source devel/setup.bash
5. cd src/
6. git clone
* https://github.com/mit-racecar/racecar.git
* https://github.com/mit-racecar/vesc.git
* https://github.com/Slamtec/rplidar_ros.git
* https://github.com/BoostChavit/Autonomous-vehicle.git

In rplidar_ros/launch
In view_rplidar_a1.launch replace with (close rviz)
```
<!--
  Used for visualising rplidar in action.  
  
  It requires rplidar.launch.
 -->
<launch>
  <include file="$(find rplidar_ros)/launch/rplidar.launch" />

  <!-- node name="rviz" pkg="rviz" type="rviz" args="-d $(find rplidar_ros)/rviz/rplidar.rviz" / -->
</launch>
```
In rplidar_a1.launch replace with
```
<launch>
  <node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="motor_pwm" value="900"/>
  <param name="serial_port"         type="string" value="/dev/ttyRplidar"/>
  <param name="scan_mode"           type="string" value="Standard"/>
  <param name="serial_baudrate"     type="int"    value="115200"/><!--A1/A2 -->
  <!--param name="serial_baudrate"     type="int"    value="256000"--><!--A3 -->
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
</launch>
```

You can change type="string" value="/dev/ttyRplidar"/> value to your rplidar device name

In mb_package/launch/mb_teleop.launch edit to 
```
<!-- -*- mode: XML -*- -->
<launch>
  <arg name="racecar_version" default="racecar-v2" />
  <arg name="run_camera" default="false"/>

  <include file="$(find rplidar_ros)/launch/view_rplidar_a1.launch"/>
  <include file="$(find mb_package)/launch/camera.launch" />
  

  <node pkg="mb_package" type="wecar.py" name="lane_detection" />

    
  <group ns="vesc">
    <!-- Spawn MUXs -->
    <include file="$(find racecar)/launch/mux.launch" />

    <!-- start electronic speed controller driver -->
    <include file="$(find racecar)/launch/includes/$(arg racecar_version)/vesc.launch.xml" >
      <arg name="racecar_version" value="$(arg racecar_version)" />
    </include>
  </group>

  <!-- start imu and laser scanner -->
  <!--include file="$(find racecar)/launch/includes/common/sensors.launch.xml" >
    <arg name="racecar_version" value="$(arg racecar_version)" />
  </include-->

  <!-- static transforms, e.g. base_link to imu -->
  <include file="$(find racecar)/launch/includes/$(arg racecar_version)/static_transforms.launch.xml" />

</launch>
``` 

7. cd ..(go to wecar_ws)
8. rosdep install --from-paths src --ignore-src -r -y
9. sudo apt-get update
9. sudo apt-get install ros-melodic-web-video-server
10. catkin_make
11. source devel/setup.bash

## Run program

run
```
roslaunch mb_package mb_teleop.launch
```
or (to see logs)
```
roslaunch mb_package mb_teleop.launch --screen
```