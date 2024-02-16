# Pre-Requisites
1. **Mavlink:** sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
2. **Mavros:** sudo apt-get install ros-${ROS_DISTRO}-mavlink
3. **VRX:** If not already, clone the gazebo_classic branch from - https://github.com/osrf/vrx/tree/gazebo_classic
4. **RobotX:** Clone the develop/master branch and follow the pre-requisites stes as mentioned in the README to build the packge.
5. **PX4**:
```
cd ~/catkin_ws/src
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
Add following into .bashrc file
```
source ~/catkin_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/catkin_ws/src/PX4-Autopilot ~/catkin_ws/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="~/catkin_ws/src/PX4-Autopilot:$ROS_PACKAGE_PATH"
export ROS_PACKAGE_PATH="~/catkin_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ROS_PACKAGE_PATH"
export GAZEBO_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH"
```
# Building the package
```
cd ~/catkin_ws/src
git clone https://github.com/sagar1327/ROS.git -b robotx
cd ..
rosdep install --from-path src --rosdistro ${ROS_DISTRO} -i -y
catkin_make --only-pkg-with-dep offboard_py
```

# Adding a UAV model in VRX environment
```
roscd minion && cd launch
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
Add the following at the end of the file.
```
<include file="$(find minion)/launch/iris_uav.launch"/>
```
![competition_course_gzclient_camera(1)-2024-02-15T00_03_53 163736](https://github.com/sagar1327/ROS/assets/125699896/db9810a0-b11d-4962-b41e-a0ed34572bac)

**Note: The above method adds the Iris UAV with a downward camera. Other models can be added using the parameter ```<arg name="vehicle" default="iris_downward_depth_camera"/>```. Check: ```~/catkin_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/``` for information about other models.**

# If you are using April Tags or Fractal Markers, follow the below steps to generate and attach it on the boat:
* https://github.com/mikaelarguedas/gazebo_models, this package can be used to generate gazebo models of different april tags.
* You can use the procided april tags in **_./images_** directory to create the model. Or You can download a different tag from https://github.com/AprilRobotics/apriltag-imgs
* Open the _**generate_markers_model.py**_ and note the path to the default gazebo directory. You can leave it to default or change the path (I recommed to change it to the _**${HOME}/catkin_ws/src/vrx/vrx_gazebo/models/aptag)**_.
* Run _**generate_markers_model.py**_ (Giving arguments is not manadatory).
* Add these at the end of _**${HOME}/catkin_ws/RobotX/minion/my_wamv/my_wamv_competition.urdf**_
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
        <mesh filename="package://vrx_gazebo/models/aptag/marker1/meshes/Marker1.dae">
        </mesh>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <mesh filename="package://vrx_gazebo/models/aptag/marker1/meshes/Marker1.dae">
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
* Change the name of the attached marker mesh file if required.    

**Note: Delete the old contents of the default gazebo directory if your are building new april tag models. And follow the above steps.**

![competition_course_gzclient_camera(1)-2024-02-15T00_19_06 955867](https://github.com/sagar1327/ROS/assets/125699896/9fc990a1-9b81-4246-9326-155730c54ebc)

* For fractal markers, download and the build the latest stable version from: https://sourceforge.net/projects/aruco/files/
* For building follow the below steps:
```
1. cd ~/catkin_ws/src
2. Downlodd the latest stable vesion. Unzip the file.
3. cd aruco-3.x.x && mkdir build && cd build
4. cmake ..
5. make -j8 #The number 8 represent the number of cores.
```
* To create your own fractal makers:
```
1. cd utils_fractal
2. ./fractal_create <output> <regions> [-s bitsize]
For example: ./fractal_create configuration.yml 10:8,14:10,6:0 -s 50
```
Where:
* output. Path of the output configuration file.
* regions. Configuring marker regions. n(f1):k(f1),n(f2):k(f2),...,n(fm),k(fm)
* bitsize. Size of each bit for the last level marker (fm). If not specified the fractal marker is normalized.

**Read the documentation to understand more about region: https://docs.google.com/document/d/1SdsOTjGdu5o8gy2Ot2FDqYDS9ALgyhOBJcJHOZBR7B4/mobilebasic**
```
3. To obatin a .png/.jpg file of the fractal marker, use: ./fractal_print_marker <output> [-c configuration] [-bs bitsize] [-noborder]
For example: ./fractal_print_marker fractal_maker.png -c configuration.yml -bs 50
```
Where:
* output. Path of the output image (jpg|png|ppm|bmp)
* configuration. Name of the marker configuration in case you want to print a default marker (If this is not specified the system uses by default FRACTAL_2L_6), or the configuration path of the file generated by the fractal_create tool.
* bitsize. Number of pixels occupied by each bit of the innermost marker (75px. by default)

**To create fractal marker gazebo models:**
```
1. Copy the generated image in ${HOME}/catkin_ws/src/gazebo_models/artags/images
2. Now follow the steps as described above to create April Tags gazebo models.
```
**Note: Make sure to delete the contents of the default gazebo directory if you are building new april tag/fractal marker gazebo models.**

![competition_course_gzclient_camera(1)-2024-02-15T00_25_00 903755](https://github.com/sagar1327/ROS/assets/125699896/eca815b0-29b8-43b4-9d78-a7a7633d6802)
