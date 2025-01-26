#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from example_interfaces.msg import Int32
from Move import Move
import time


class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        
        timer_period = 0.05
        
        self.subscription = self.create_subscription(
            Int32,
            '/direction',
            self.listener_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/kalman_filter',
            self.listener_kalman_callback,
            10
        )

        #self.pos = np.array([0,0,0,0], float)
        self.pos = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], float)

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 100)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.nb =0.0

        self.pair1_avance = True
        self.avancer = Move()
        self.Y = 0
        self.Z = 90 #95

        self.i = 0
        
        self.message = 0
        self.message_kalman = 0
        self.time_avancer_droit = time.time()
        
    def listener_callback(self, msg):
        self.message = msg.data

    def listener_kalman_callback(self, msg):
        max_value = 0.15
        roll_kalman = msg.data[0]*math.pi/180 -0.1
        if roll_kalman >0.08:
            self.avancer.roll += 0.01
        elif roll_kalman <-0.08:
            self.avancer.roll -= 0.01
        self.avancer.roll = max(-max_value, min(max_value, self.avancer.roll))

        pitch_kalman = msg.data[1]*math.pi/180 + 0.1
        if pitch_kalman >0.08:
            self.avancer.pitch += 0.01
        elif pitch_kalman <-0.08:
            self.avancer.pitch -= 0.01
        self.avancer.pitch = max(-max_value, min(max_value, self.avancer.pitch))
        print("value start : ", roll_kalman, pitch_kalman)

        

    def timer_callback(self):
        self.avancer.debout()
        self.pos = self.avancer.pos_ang
        pos_array = Float64MultiArray(data=self.pos) 
        self.pub_pos.publish(pos_array)


if __name__ == '__main__':

    rclpy.init(args=None)
    
    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(50)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

