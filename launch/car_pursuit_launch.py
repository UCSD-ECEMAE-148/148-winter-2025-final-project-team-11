from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('robocar_visual_pursuit_pkg')
    package_src = '/home/projects/ros2_ws/src/robocar_visual_pursuit_pkg/robocar_visual_pursuit_pkg'
    
    # Path to the model file and config files
    model_path = os.path.join('/home/projects/ros2_ws/src/robocar_visual_pursuit_pkg/models', 'v8-weights_openvino_2022.1_5shave-fast.blob')
    custom_config = os.path.join(package_dir, 'config', 'custom_params.yaml')
    default_config = os.path.join(package_dir, 'config', 'default_params.yaml')
    
    # Use custom config if it exists, otherwise use default
    config_file = custom_config if os.path.exists(custom_config) else default_config
    
    return LaunchDescription([
        # Parameter tuner node
        # ExecuteProcess(
        #     cmd=['python3', os.path.join(package_src, 'parameter_tuner_node.py'), config_file],
        #     name='parameter_tuner_node',
        #     output='screen'
        # ),
        
        # Car detection node
        ExecuteProcess(
            cmd=['python3', os.path.join(package_src, 'car_detection_node.py'), config_file, model_path],
            name='car_detection_node',
            output='screen'
        ),
        
        # Lane guidance node
        ExecuteProcess(
            cmd=['python3', os.path.join(package_src, 'lane_guidance_node.py'), config_file],
            name='lane_guidance_node',
            output='screen'
        ),

        # LED controller node
        ExecuteProcess(
            cmd=['python3', os.path.join(package_src, 'led_controller_node.py'), config_file],
            name='led_controller_node',
            output='screen'
        ),
        
        # VESC node from the actuator package
        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen'
        )

        # # Vesc ODOM node from the actuator package 
        # Node(
        #     package='ucsd_robocar_actuator2_pkg',
        #     executable='vesc_odom_node',
        #     name='vesc_odom_node',
        #     output='screen'
        # )
    ])
