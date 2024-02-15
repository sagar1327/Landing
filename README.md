# Pre-Requisites

1. **Mavlink:** Clone the updated master branch from - https://github.com/mavlink/mavlink
2. **Mavros:** Clone the updated master branch from - https://github.com/mavlink/mavros
3. **VRX:** If not already, clone the gazebo_classic branch from - https://github.com/osrf/vrx/tree/gazebo_classic
4. **RobotX:** Clone the develop/master branch and follow the pre-requisites stes as mentioned in the README to build the packge.

## If you are using April tags or Fractal Markers, follow the below steps to generate and attach it on the boat:
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
