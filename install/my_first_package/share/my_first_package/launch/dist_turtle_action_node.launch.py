from launch import LaunchDescription
from launch_ros.actions import Node #, ExecuteProcess

def generate_launch_description():
    # 파라미터 바꾸는 부분
    my_launch = LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
        parameters=[
            {"background_r": 255},
            {"background_b": 192},
            {"background_g": 203}
        ]
    )

    dist_turtle_action_node = Node(
        package='my_first_package',
        executable='dist_turtle_action_server',
        output='screen'
    )

    my_launch.add_action(turtlesim_node)
    my_launch.add_action(dist_turtle_action_node)

    return my_launch
