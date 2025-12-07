---
sidebar_position: 3
sidebar_label: '3. URDF Basics'
---

# Chapter 3: URDF for Humanoid Robots

The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all elements of a robot. This includes the robot's links (its physical parts), joints (which connect the links), sensors, and visual appearance.

## Core URDF Components

A URDF file is made up of a few key tags:

-   `<robot>`: The root tag for the entire robot model.
-   `<link>`: Represents a rigid part of the robot, like a limb or a chassis. A link has properties like its visual appearance (`<visual>`), collision properties (`<collision>`), and inertial properties (`<inertial>`).
-   `<joint>`: Connects two links together. A joint defines the relationship between a `parent` link and a `child` link. It also specifies the `type` of motion allowed (e.g., `fixed`, `revolute`, `continuous`, `prismatic`).

## Example: A Simple Humanoid URDF

Let's examine a simple URDF file that describes a basic humanoid shape with a base, a torso, and a head.

File: `examples/ros2_basics_examples/models/simple_humanoid.urdf`
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="torso_to_head" type="continuous">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

**Code Breakdown**:
1.  **Links**: We define three `<link>` elements: `base_link`, `torso`, and `head`. Inside each, the `<visual>` tag describes what it looks like. We use simple shapes like `<box>` and `<sphere>` for the geometry.
2.  **Joints**: We define two `<joint>` elements to connect the links.
    -   `base_to_torso`: This is a `fixed` joint, meaning the torso is rigidly attached to the base. The `<origin>` tag specifies the offset of the child link relative to the parent.
    -   `torso_to_head`: This is a `continuous` joint, which allows for infinite rotation. The `<axis>` tag specifies that the head can rotate around the Z-axis.

## Visualizing the URDF with RViz2

RViz2 is a powerful 3D visualization tool in ROS 2. You can use it to view your URDF models. You will also need the `robot_state_publisher` package, which reads the URDF and publishes the robot's structure as a transformation tree.

### How to Visualize the Model

1.  **Ensure `robot_state_publisher` is installed**:
    ```bash
    sudo apt-get install ros-humble-robot-state-publisher
    ```

2.  **Create a Launch File**: To make visualization easy, we use a launch file. Create a file named `display.launch.py` in your `ros2_basics_examples` package.

    ```python
    import os
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        pkg_share = get_package_share_directory('ros2_basics_examples')
        urdf_file = os.path.join(pkg_share, 'models/simple_humanoid.urdf')

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                arguments=[urdf_file]),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(pkg_share, 'rviz/display.rviz')])
        ])
    ```
    *(Note: You would also need to create a basic `display.rviz` configuration file and add the launch file to `setup.py`)*

3.  **Launch and View**:
    After building your package, you can run the launch file:
    ```bash
    ros2 launch ros2_basics_examples display.launch.py
    ```
    This will open RViz2, and you should see your simple humanoid model displayed.
