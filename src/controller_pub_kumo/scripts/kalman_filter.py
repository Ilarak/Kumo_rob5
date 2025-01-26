#!/usr/bin/env python3

import time
import math
import numpy as np
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Clear the port and initialize serial connection
port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=1, writeTimeout=1)

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/kalman_filter', 10)

    def publish_data(self, data):
        msg = Float32MultiArray()
        msg.data = data.tolist()
        self.publisher_.publish(msg)

rclpy.init()
node = SensorDataPublisher()

D = -13.2
g = 9.81
Ts = 0.1

t = []
est_values = np.zeros((3, 1))
startime = time.time()
x_k_k = np.array([[0], [0], [0]])
x_k_kMoinsUn = np.array([[0], [0], [0]])

P_k_kMoinsUn = np.diag([180, 180, 180])
H_k = np.eye(3)

# Variance values
Vx, Vy, Vz = 1**2, 1**2, 1**2
Vx_acc, Vy_acc, Vz_mag = 2**2, 2**2, 2**2

R_k = np.diag([Vx_acc, Vy_acc, Vz_mag])
I = np.eye(3)

while True:
    ts_time = time.time()

    # Read data from serial port
    data = port.readline().decode('utf-8').strip()
    numbers = [float(num) for num in data.split()]
    if(len(numbers)>5):
        p, q, r, ax, ay, az, mx, my, mz = numbers

        # Gyroscope output for the model
        pqr = np.array([[p], [q], [r]])

        # Angle calculations from accelerometer and magnetometer
        in_atanxy = ay / az if az != 0 else 0
        if abs(in_atanxy % (math.pi / 2)) < 1e-4:
            in_atanxy = math.pi / 2 - 0.0001

        phi_est = math.degrees(math.atan2(ay, az))

        in_asin = ax / g
        in_asin = max(min(in_asin, 1 - 0.0001), -1 + 0.0001)
        theta_est = math.degrees(math.asin(in_asin))

        if mx != 0 and my != 0 and mz != 0:
            in_atanphi_num = my * math.cos(math.radians(x_k_k[0, 0])) - mz * math.sin(math.radians(x_k_k[0, 0]))
            in_atanphi_den = (mx * math.cos(math.radians(x_k_k[1, 0])) +
                            my * math.sin(math.radians(x_k_k[1, 0])) * math.sin(math.radians(x_k_k[0, 0])) +
                            mz * math.sin(math.radians(x_k_k[1, 0])) * math.cos(math.radians(x_k_k[0, 0])))

        psi_est = D - math.degrees(math.atan2(in_atanphi_num, in_atanphi_den))

        currentTime = time.time() - startime
        t.append(currentTime)
        est_values[:, 0] = [phi_est, theta_est, psi_est]

        K_k = P_k_kMoinsUn @ H_k.T @ np.linalg.inv(H_k @ P_k_kMoinsUn @ H_k.T + R_k)
        x_k_k = x_k_kMoinsUn + K_k @ (est_values - H_k @ x_k_kMoinsUn)

        t_p = math.tan(math.radians(x_k_k[1, 0]))
        c_p = math.cos(math.radians(x_k_k[1, 0]))
        if abs(c_p) < 1e-4:
            c_p = 0.001

        c_r = math.cos(math.radians(x_k_k[0, 0]))
        s_r = math.sin(math.radians(x_k_k[0, 0]))

        F = np.array([
            [1, t_p * s_r, t_p * c_r],
            [0, c_r, -s_r],
            [0, s_r / c_p, c_r / c_p]
        ])

        phi = I + F * Ts

        Q1_1 = Ts * (Vx * (1 + Ts + Ts**2 / 3) + (Ts**2 / 3) * t_p**2 * (s_r**2 * Vy + c_r**2 * Vz))
        Q1_2 = Ts**2 * t_p * s_r * ((0.5 + c_r * Ts / 3) * Vy - c_r * Ts / 3 * Vz)
        Q1_3 = t_p * Ts**2 * ((Vy * Ts * s_r**2) / 3 + Vz * c_r * (c_r * Ts / 3 + c_p * 0.5)) / c_p
        Q2_2 = Ts * (Vy * (c_r**2 * Ts**2 / 3 + c_r * Ts + 1) + Ts**2 * s_r**2 * Vz / 3)
        Q2_3 = Ts**2 * s_r * (c_r * Ts * Vy / 3 + 0.5 * Vy - c_r * Vz * Ts / 3 - 0.5 * c_p * Vz) / c_p
        Q3_3 = Ts * (Ts**2 * s_r**2 / (3 * c_p**2) * Vy + (Ts**2 * c_r**2 / (3 * c_p**2) + c_r / c_p + 1) * Vz)

        Q_k = np.array([
            [Q1_1, Q1_2, Q1_3],
            [Q1_2, Q2_2, Q2_3],
            [Q1_3, Q2_3, Q3_3]
        ])

        P_k_k = (I - K_k @ H_k) @ P_k_kMoinsUn
        P_kPlusUn_k = phi @ P_k_k @ phi.T + Q_k

        x_kPlusUn_k = x_k_k + F @ pqr * Ts

        node.publish_data(x_k_k.ravel())

        x_k_kMoinsUn = x_kPlusUn_k
        P_k_kMoinsUn = P_kPlusUn_k

    elapsed_time = time.time() - ts_time
    pause_time = Ts - elapsed_time
    if pause_time > 0:
        time.sleep(pause_time)