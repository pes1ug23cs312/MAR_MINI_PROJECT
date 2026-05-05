from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():

    world = os.path.expanduser(
        '~/line_follow_ws/src/line_follower_robot/worlds/line_track_mission.world')

    urdf = os.path.expanduser(
        '~/line_follow_ws/src/line_follower_robot/urdf/robot.urdf')

    rviz_config = os.path.expanduser(
        '~/line_follow_ws/src/line_follower_robot/rviz/robot_view.rviz')

    with open(urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # 🔹 Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s',
                 'libgazebo_ros_factory.so', world],
            output='screen'),

        # 🔹 Spawn robot
        TimerAction(period=5.0, actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'robot', '-file', urdf,
                           '-x', '0.0', '-y', '2.05', '-z', '0.05',
                           '-Y', '-1.5708'],
                output='screen'),
        ]),

        # 🔹 Core nodes
        TimerAction(period=8.0, actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen'),

            Node(
                package='line_follower_robot',
                executable='line_detector',
                output='screen'),

            Node(
                package='line_follower_robot',
                executable='unified_controller',
                output='screen'),
        ]),

        # 🔥 ADD THIS BLOCK (RViz launch)
        TimerAction(period=10.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            ),
        ]),
    ])