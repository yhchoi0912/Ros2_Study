import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist 
from my_first_packgae_msgs.action import DistTurtle
from my_first_package.my_subscriber import TurtlesimSubscriber


import time
import math

class Trutlesub_Action(TurtlesimSubscriber):
    def __init__(self, ac_server):
        super().__init__()
        self.ac_server = ac_server

    def callback(self, msg):
        self.ac_server.current_pose = msg
    

class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0
        self.is_first_time = True
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel',10)
        self.action_sercver = ActionServer(self, DistTurtle, 'dist_turtle', self.execute_callback)

    def calc_diff_pose(self):
        if self.is_first_time:
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            self.is_first_time = False
        
        diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x)**2 +\
                               (self.current_pose.y - self.previous_pose.y)**2)

        self.previous_pose = self.current_pose

        return diff_dist

    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback()
        
        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remined_dist = goal_handle.request.dist - self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.publisher.publish(msg)
            time.sleep(0.01)

            if feedback_msg.remined_dist < 0.2:
                break
        
        goal_handle.succeed()
        result = DistTurtle.Result()

        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_dist = self.total_dist

        self.total_dist = 0
        self.is_first_time = True

        return result


    #def __init__(self):
    #    super().__init__('dist_turtle_action_server')
    #    self.action_server = ActionServer(
    #        self, DistTurtle, 'dist_turtle', self.execute_callback)
    #

    #def execute_callback(self, goal_handle): # send_goal을 하면 수행됨
    #    feedbak_msg = DistTurtle.Feedback() # 피드백을 수행할 객체 생성
    #    for i in range(0,10):
    #        feedbak_msg.remined_dist = float(i)
    #        goal_handle.publish_feedback(feedbak_msg) # 피드백 메시지 보냄
    #        time.sleep(0.5)

    #    goal_handle.succeed()
    #    result = DistTurtle.Result()
    #    return result

def main(args=None):
    rp.init(args=args)

    executor = MultiThreadedExecutor()

    ac = DistTurtleServer()
    sub = Trutlesub_Action(ac_server = ac)

    executor.add_node(ac)
    executor.add_node(sub)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        sub.destroy_node()
        ac.destroy_node()
        rp.shutdown()
    
    
    #dist_turtle_action_server = DistTurtleServer()
    #rp.spin(dist_turtle_action_server)

if __name__ == "__main__":
    main()