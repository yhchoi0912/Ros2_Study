from launch import LaunchDescription
from launch_ros.actions import Node

# 한 터니멀에서 여러 노드를 한번에 실행

def generate_launch_description(): # 엔트리 포인트

    return LaunchDescription(
        [   # 여러노드 한번에 실행하는 부분
            Node(
                namespace="turtlesim", package="turtlesim", 
                executable="turtlesim_node", output="screen"),
            Node(
                namespace="pub_cmd_vel", package="my_first_package",
                executable="my_publisher", output="screen")
        ]
    )