#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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
        self.time_avancer_droit = time.time()
        
    def listener_callback(self, msg):
        self.message = msg.data
        #print(msg.data)
        #2 gauche, 1 droite, 0 avant/pas d'obstacles
        

    def timer_callback(self):
        if (self.message == 2):
            self.time_avancer_droit = time.time()
            self.avancer.incr_y = 5
            self.Y,self.Z, self.pair1_avance = self.avancer.tourner_gauche(self.Y, self.Z, self.pair1_avance)
            print("tourner à gauche")
            #gauche
        elif (self.message == 1):
            self.time_avancer_droit = time.time()
            self.avancer.incr_y = 5
            self.Y,self.Z, self.pair1_avance = self.avancer.tourner_droite(self.Y, self.Z, self.pair1_avance)
            print("tourner à droite")
            #droite
        else :
            time_now = time.time()
            print(self.time_avancer_droit-time_now)
            if(time_now-self.time_avancer_droit>1.5):
                self.avancer.incr_y = 10
                print("speed augmenté")
            self.Y,self.Z, self.pair1_avance = self.avancer.avancer(self.Y, self.Z, self.pair1_avance)
            

        #self.Y,self.Z, self.pair1_avance = self.avancer.reculer(self.Y, self.Z, self.pair1_avance)
        #self.avancer.stars()
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

