"""ROS2 launch file for PMSM FOC system."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    foc_node = Node(
        package='pmsm_ros2',
        executable='pmsm-foc-node',
        name='pmsm_foc_controller',
        parameters=[{
            'motor.Rs': 0.5,
            'motor.Ld': 1.4e-3,
            'motor.Lq': 1.4e-3,
            'motor.flux_pm': 0.0577,
            'motor.pole_pairs': 4,
            'controller.Kp_id': 5.0,
            'controller.Ki_id': 1000.0,
            'controller.Kp_iq': 5.0,
            'controller.Ki_iq': 1000.0,
            'controller.Kp_speed': 0.5,
            'controller.Ki_speed': 10.0,
            'controller.iq_max': 10.0,
            'controller.id_max': 10.0,
            'inverter.Vdc': 24.0,
            'inverter.fsw': 20000.0,
        }],
        output='screen',
    )

    matlab_bridge = Node(
        package='pmsm_ros2',
        executable='pmsm-matlab-bridge',
        name='matlab_bridge',
        parameters=[{
            'bridge_mode': 'file',
            'data_dir': '/tmp/pmsm_bridge',
        }],
        output='screen',
    )

    return LaunchDescription([foc_node, matlab_bridge])
