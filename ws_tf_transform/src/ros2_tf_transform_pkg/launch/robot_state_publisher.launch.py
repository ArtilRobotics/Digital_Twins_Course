import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'ros2_tf_transform_pkg'
    
    package_share_dir = get_package_share_directory(pkg_name)
    launch_dir = os.path.join(package_share_dir, 'launch')
    resources_dir = os.path.join(package_share_dir, 'resources')
    config_dir = os.path.join(package_share_dir, 'config')
    
    file_subpath = os.path.join(resources_dir, 'urdf', 'robot.urdf.xacro')
    path_rviz = os.path.join(config_dir, 'rviz_config.rviz')
    
    # Specify the path to the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        path_rviz
    )


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    # Node jointstate publisher
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    # Configure the RViz node
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])
