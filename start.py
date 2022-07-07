from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            namespace='mavros',
            executable='mavros_node',
            name='mavros_node'
        ),
        Node(
            package='brain',
            namespace='brain',
            executable='state_machine',
            name='state_machine'
        ),
        #Node(
        #    package='test',
        #    namespace='test',
        #    executable='test_node',
        #    name='test_node'
        #),
        Node(
            package='sub',
            executable='movement',
            name='movement_node',
            #remappings=[
            #    ('/input/pose', '/turtlesim1/turtle1/pose'),
            #    ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            #]
        )
    ])
