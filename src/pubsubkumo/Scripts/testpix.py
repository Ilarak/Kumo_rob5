#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.msg import Int32
import cv2
import numpy as np

tresholdmin = 170
tresholdmax = 400

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.fx = 605.0  # Paramètre de la caméra (focale x)
        self.fy = 605.0  # Paramètre de la caméra (focale y)
        self.cx = 320.0  # Coordonnée x du centre de l'image
        self.cy = 240.0  # Coordonnée y du centre de l'image
        self.publisher_ = self.create_publisher(Int32, 'direction', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)

        # Récupération des dimensions de l'image
        height, width = depth_array.shape
        
        values = np.linspace(0, 1, width//2, endpoint=False)
        vls = np.concatenate((values, values[::-1]))

        clipped_image = np.where((depth_array >= tresholdmin) & (depth_array <= tresholdmax), depth_array, 0)
        weighted_img = clipped_image * vls

        # Diviser l'image en deux parties (gauche et droite)
        left_half = depth_array[:, :width//2]
        right_half = depth_array[:, width//2:]

        # Appliquer le seuil pour filtrer les pixels valides
        valid_left = np.where((left_half >= tresholdmin) & (left_half <= tresholdmax), 1, 0)
        valid_right = np.where((right_half >= tresholdmin) & (right_half <= tresholdmax), 1, 0)

        # Compter les pixels valides
        count_left = np.sum(valid_left)
        count_right = np.sum(valid_right)

        print(f"Nombre de pixels valides à gauche : {count_left}")
        print(f"Nombre de pixels valides à droite : {count_right}")

        # Déterminer la direction en fonction des pixels valides
        if count_left > count_right:
            print("Plus de pixels valides à gauche, aller à droite")
            self.direction = 1  # Aller à droite
        elif count_right > count_left:
            print("Plus de pixels valides à droite, aller à gauche")
            self.direction = 2  # Aller à gauche
        else:
            print("Égalité des pixels, tout droit")
            self.direction = 0  # Aller tout droit

        # Choisir un pixel d'intérêt (par exemple, le centre de l'image)
        u = int(width / 2)
        v = int(height / 2)

        Z = depth_array[v, u]  # Profondeur à ce pixel
        if Z > 0:  # Vérifiez que la profondeur est valide
            distance = Z  # Convertir la profondeur en centimètres
            print(f"Distance à l'objet (centre de l'image) : {distance:.2f} mm")
        else:
            print(f"Profondeur non valide à ce pixel (estimé: {Z:.2f} mm)")

        # Ajouter un point rouge au centre de l'image
        color_depth_image = cv2.cvtColor(depth_array, cv2.COLOR_GRAY2BGR)  # Convertir en image couleur
        cv2.circle(color_depth_image, (u, v), 5, (0, 0, 255), -1)  # Dessiner un cercle rouge au centre
        
        # Afficher l'image de profondeur
        cv2.imshow('Depth Image', weighted_img)
        cv2.waitKey(1)

    def timer_callback(self):
        msg = Int32()
        msg.data = self.direction
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing : {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
