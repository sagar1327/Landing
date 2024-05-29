# Pre-Requisites
  1. **Mavlink:** sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
  2. **Mavros:** sudo apt-get install ros-${ROS_DISTRO}-mavlink
  3. **VRX:** If not already, clone the gazebo_classic branch from - https://github.com/osrf/vrx/tree/gazebo_classic
  4. **RobotX:** Clone the develop/master branch and follow the pre-requisites stes as mentioned in the README of the RobotX repo to build the packges.
  5. **PX4**:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```
Note: Clone and build in your $HOME directory.

Add following into .bashrc file:
```
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="$HOME/PX4-Autopilot:$ROS_PACKAGE_PATH"
export ROS_PACKAGE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH"
export GAZEBO_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH"
```

Use ```make clean``` before rebuilding.

# Building the package
```
cd ~/catkin_ws/src
git clone https://github.com/sagar1327/ROS.git
cd ..
rosdep install --from-path src/offboard_py --rosdistro ${ROS_DISTRO} -i -y
catkin_make --only-pkg-with-dep offboard_py
```

# Adding a UAV model in VRX environment
```
roscd minion/launch
gedit iris_uav.launch
```
Paste the following:
```
<?xml version="1.0"?>
<launch>
      <!-- Posix SITL environment launch script -->
      <!-- launches PX4 SITL, Gazebo environment, and spawns vehicle -->
      <!-- vehicle pose -->
      <arg name="x" default="-828.787354"/>
      <arg name="y" default="240.884674"/>
      <arg name="z" default="2.479315"/>
      <arg name="R" default="0"/>
      <arg name="P" default="0"/>
      <arg name="Y" default="0"/>
      <!-- vehicle model and world -->
      <arg name="est" default="ekf2"/>
      <arg name="vehicle" default="iris_downward_depth_camera"/>
      <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/>
      <env name="PX4_SIM_MODEL" value="gazebo-classic_$(arg vehicle)" />

      <!-- PX4 configs -->
      <arg name="interactive" default="true"/>
      <!-- PX4 SITL -->
      <arg unless="$(arg interactive)" name="px4_command_arg1" value="-d"/>
      <arg     if="$(arg interactive)" name="px4_command_arg1" value=""/>
      <node name="sitl" pkg="px4" type="px4" output="screen"
            args="$(find px4)/build/px4_sitl_default/etc -s etc/init.d-posix/rcS $(arg px4_command_arg1)" required="true"/>

      <!-- gazebo model -->
      <node name="$(anon vehicle_spawn)" pkg="gazebo_ros" type="spawn_model" output="screen" args="-sdf -file $(arg sdf) -model $(arg vehicle) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"/>

      <!-- MAVROS configs -->
      <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
      <arg name="respawn_mavros" default="false"/>
      <!-- MAVROS -->
      <include file="$(find mavros)/launch/px4.launch">
        <!-- GCS link is provided by SITL -->
        <arg name="gcs_url" value=""/>
        <arg name="fcu_url" value="$(arg fcu_url)"/>
        <arg name="respawn_mavros" value="$(arg respawn_mavros)"/>
      </include>
</launch>
```
Open the sydney.launch file.
```
roscd minion && cd launch
gedit sydney.launch
```
Add the following at the end.
```
<include file="$(find minion)/launch/iris_uav.launch"/>
```
![competition_course_gzclient_camera(1)-2024-02-15T00_03_53 163736](https://github.com/sagar1327/ROS/assets/125699896/db9810a0-b11d-4962-b41e-a0ed34572bac)

**Note: The above method adds the Iris UAV with a downward camera. Other models can be added using the parameter ```<arg name="vehicle" default="iris_downward_depth_camera"/>```. Check: ```~/catkin_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/``` for information about other models.**

# If you are using April Tags, follow the below steps to generate and attach it on the boat:
* ```
  git clone https://github.com/mikaelarguedas/gazebo_models.git
  git clone https://github.com/AprilRobotics/apriltag-imgs.git
  cd apriltag-imgs
  convert <small_marker>.png -scale 1700% <big_marker>.png
  ```
* Here <small_marker> is the path of the apriltag, ex - tag36h11/tag36_11_00000. And <big_marker> is the output file name, ex - Marker0.
* Delete all the images from _gazebo_models/ar_tags/images_ and move the apriltag, resized in the previous step, there.
  * ```
    cd gazebo_models/ar_tags/images
    cd rm *
    mv apriltag-imgs/Marker0 .
    ```
* Open the _generate_markers_model.py_ script.
   * ```
     cd gazebo_models/ar_tags/script
     gedit generate_markers_model.py
     ```
* Note the path to the default gazebo directory. You can leave it to default or change the path (Recommended: _${HOME}/catkin_ws/src/vrx/vrx_gazebo/models/artag_).
* Optional: If chaning to the recommended path
  * ```
    roscd vrx_gazebo/models
    mkdir artag
    ```
* Run _generate_markers_model.py_.
  * ```./generate_markers_model.py```
* Add the apriltag model on the boat.
  * ```
    roscd minion/my_wamv
    gedit my_wamv_competition.urdf
    ```
  * Copy the following and paste at the end of the file.
      ```
        <!--AR tag-->
        <link name="wamv/aptag">
          <inertial>
            <mass value="0.01"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
          </inertial>
          <visual> 
            <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
            <geometry>
              <mesh filename="package://vrx_gazebo/models/aptag/marker0/meshes/Marker0.dae">
              </mesh>
            </geometry>
          </visual>
          <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/> 
            <geometry>
              <mesh filename="package://vrx_gazebo/models/aptag/marker0/meshes/Marker0.dae">
              </mesh>
            </geometry>
          </collision>
        </link>
        <joint name="wamv/aptag_joint" type="fixed">
            <parent link="wamv/base_link"/>
            <child link="wamv/aptag"/>
            <origin rpy="0 1.5707963 1.5707963" xyz="0.035 -0.25 1.76"/>
        </joint>
      ```
* Similarly, different/multiple apriltags can be generated and added to the wamv.   

**Note: Delete the old contents of the default gazebo directory if your are building new april tag models. And follow the above steps.**

![competition_course_gzclient_camera(1)-2024-02-15T00_19_06 955867](https://github.com/sagar1327/ROS/assets/125699896/9fc990a1-9b81-4246-9326-155730c54ebc)
