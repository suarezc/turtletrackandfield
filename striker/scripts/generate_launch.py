  
template_start = """<launch>
  <!-- modified from the empty world launch file -->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="0.0"/>
  <arg name="y_pos" default="0.0"/>
  <arg name="z_pos" default="0.0"/>

  <env name="GAZEBO_MODEL_PATH" value="$GAZEBO_MODEL_PATH:$(find striker)/models"/>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find striker)/worlds/three_lane.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
"""

template_end = "</launch>"

launch_file = template_start

for i in range(3):
  robot = f"""
  <group ns="robot{i}">
    <param name="tf_prefix" value="robot{i}_tf" />
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />

    <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model lane_{i}_robot -x $(arg x_pos) -y {2 * i} -z $(arg z_pos) -param robot_description" />
  </group>
  """
  launch_file += robot

launch_file += template_end

print(launch_file)
