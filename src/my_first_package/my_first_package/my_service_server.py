import rclpy as rp
from rclpy.node import Node
import time
import numpy as np


from my_first_packgae_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Spawn

class MultiSpawning(Node):
    def __init__(self):
        super().__init__('multi_spawn')
        self.server = self.create_service(MultiSpawn, 'my_multi_spwan', self.callback_service) # 받는 놈
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute') # 보내는 놈
        self.req_teleport = TeleportAbsolute.Request() # 서비스 요청 객체 생성
        self.req_spawn = Spawn.Request()
        self.spawn = self.create_client(Spawn, '/spawn')
        # 원의 중심
        self.center_x = 5.54
        self.center_y = 5.54

    def calc_position(self, n ,r):
        gap_theta = 2*np.pi / n
        theta = [gap_theta * i for i in range(n)]
        x = [r *np.cos(th) for th in theta]
        y = [r *np.sin(th) for th in theta]

        return x,y, theta

    def callback_service(self, request, response):
        x, y, theta = self.calc_position(request.num, 3)

        for i in range(len(theta)):
            self.req_spawn.x = x[i] + self.center_x
            self.req_spawn.y = y[i] + self.center_y
            self.req_spawn.theta = theta[i]
            self.spawn.call_async(self.req_spawn)
            time.sleep(0.1)

        response.x = x
        response.y = y
        response.theta = theta

        return response

        #self.req_teleport.x = 1.
        #self.teleport.call_async(self.req_teleport) # 실제로 보냄

        #print("request: ", request)
        #response.x = [1.0, 2.0, 3.0]
        #response.y = [10., 10.]
        #response.theta = [100., 200., 300.]

        return response

def main(args=None):
    rp.init()
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()


if __name__ == '__main__':
    main()        