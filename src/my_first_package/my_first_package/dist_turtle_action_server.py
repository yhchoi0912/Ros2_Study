import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import SetParametersResult # 파라미터의 변경 정보를 실시간으로 알고 싶은 경우
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange # rqt에서 슬라이드 바를 만들기 위해 필요

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
        self.ac_server.current_pose = msg # ac_server이게 객체 선언시 인자로 들어오면 TurtlesimSubscriber여기서 받은 msg를 인자로 넘겨준다
    

class DistTurtleServer(Node):
    def __init__(self):
        super().__init__('dist_turtle_action_server')
        self.total_dist = 0
        self.is_first_time = True
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel',10)
        self.action_sercver = ActionServer(self, DistTurtle, 'dist_turtle', self.execute_callback)

        # start log
        self.get_logger().info("Dist turtle action server is started!!")

        param_desc_quantile = ParameterDescriptor(
            description = 'quantile_time_description',
            floating_point_range = [FloatingPointRange(
                                    from_value=0.0,
                                    to_value=1.0,
                                    step=0.01)]
        )

        self.declare_parameter('quantile_time', 0.75, param_desc_quantile) # 파라미터의 선언, 범위 제한치 설정값을 같이 선언
        self.declare_parameter('almost_goal_time', 0.95) # 파라미터의 선언
        
        # 입력 받은 파라미터를 코드에서 사용할 수 있도록 변수에 저장
        (quantile_time, almost_goal_time) = self.get_parameters(['quantile_time', 'almost_goal_time'])

        #print("quantile_time and almost_goal_time: ", quantile_time.value, almost_goal_time.value)
        
        self.quantile_time = quantile_time.value
        self.almost_goal_time = almost_goal_time.value

        # 실시간으로 파라미터의 변화를 보기 위한 코드 
        self.add_on_set_parameters_callback(self.parameter_callback) # 파라미터가 수정되면 바로 반응

        output_msg = "quantile time is: " + str(self.quantile_time) + ", "
        output_msg = output_msg + "and almost goal time is: " + str(self.almost_goal_time) + "."

        self.get_logger().info(output_msg)

    def parameter_callback(self, params):
        for param in params:
            print(param.name, " is change to ", param.value)

            if param.name == "quantile_time":
                self.quantile_time = param.value
            if param.name == "almost_goal_time":
                self.almost_goal_time = param.value

        output_msg = "quantile time is: " + str(self.quantile_time) + ", "
        output_msg = output_msg + "and almost goal time is: " + str(self.almost_goal_time) + "."
        self.get_logger().info(output_msg)

        #print("quantile time and almost goal time is ", self.quantile_time, self.almost_goal_time)


        return SetParametersResult(successful=True)



    def calc_diff_pose(self):
        if self.is_first_time:
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            self.is_first_time = False
        
        diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x)**2 +\
                               (self.current_pose.y - self.previous_pose.y)**2)

        self.previous_pose = self.current_pose

        return diff_dist

    def execute_callback(self, goal_handle): # goal은 커멘드라인에서 보냄
        feedback_msg = DistTurtle.Feedback()
        
        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remined_dist = goal_handle.request.dist - self.total_dist
            goal_handle.publish_feedback(feedback_msg) # 커멘드라인에서 send_gaol했으면 커멘드 라인으로 보냄
            self.publisher.publish(msg) # 터틀심으로 감

            temp = feedback_msg.remined_dist - goal_handle.request.dist + self.quantile_time
            temp = abs(temp)

            if temp < 0.02:
                output_msg = "The Turtle passes the " + str(self.quantile_time) + "point."
                output_msg = output_msg + " : " + str(temp)
                self.get_logger().info(output_msg)

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