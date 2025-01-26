#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import math


class Communication(Node):

    def __init__(self):
        super().__init__('communication')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/forward_position_controller/commands',
            self.listener_callback,
            10
        )
        self.i=0
        self.subscription
        self.arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1, writeTimeout=1)
        self.offset = [20,0,400, #0-2
                       0,40,400,
                       -40,-100,380,
                       -20,60,350,#16-18
                       -60,-50,380,
                       0,-20,420]

        self.toutPin = [0,1,2,4,5,6,12,13,14,16,17,18,20,21,22,24,25,26]

    def listener_callback(self, msg):
        message =""
        for i in range(18):
            if(i%3==2):
                message+="#"+str(self.toutPin[i])+" P"+str((self.remap(-msg.data[i])-self.offset[i]))+" "
            else:
                message+="#"+str(self.toutPin[i])+" P"+str((self.remap(msg.data[i])-self.offset[i]))+" "

        message+="T50"
        print(message)
        self.arduino.write((message+'\r').encode())
        self.i+=1


    def remap(self, value, orig_min=-math.pi/2, orig_max=math.pi/2, target_min=500, target_max=2500):
        return int((value - orig_min) / (orig_max - orig_min) * (target_max - target_min) + target_min)


    def receive_data(self):
        data = self.arduino.readline().decode('utf-8').strip()
        #print("data : ",data)

    def send_message(self):
        message = "#21 P1600 T1000 <cr>\n"
        print("message")
        self.arduino.write(message.encode(encoding="utf-8"))
        

def main(args=None):
    rclpy.init(args=args)

    listener = Communication()

    rclpy.spin(listener)

if __name__ == '__main__':
    main()
