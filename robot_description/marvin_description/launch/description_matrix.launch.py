from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ARM_MODELS = [
    'm6_696',
    'm3_615',
    'm6s_lite_615',
    'm6s_lite_680',
    'm6s_lite_750',
    'm6s_lite_809',
]
BASE_MODELS = ['legacy', 'new']


def generate_launch_description():
    package_path = FindPackageShare('marvin_description')
    model_path = PathJoinSubstitution(
        [package_path, 'urdf', 'marvin_CCS_matrix.urdf.xacro']
    )
    rviz_path = PathJoinSubstitution([package_path, 'launch', 'urdf.rviz'])
    base_model = LaunchConfiguration('base_model')
    arm_model = LaunchConfiguration('arm_model')

    robot_description = ParameterValue(
        Command([
            'xacro ', model_path,
            ' base_model:=', base_model,
            ' arm_model:=', arm_model,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_model',
            default_value='legacy',
            choices=BASE_MODELS,
            description='Base model',
        ),
        DeclareLaunchArgument(
            'arm_model',
            default_value='m6_696',
            choices=ARM_MODELS,
            description='Matched left/right arm model',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz2',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=rviz_path,
            description='RViz2 configuration file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
