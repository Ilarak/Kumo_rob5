import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.msg import Int32
from sklearn.linear_model import RANSACRegressor
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def distance_to_plane(point, plane_params):
    a, b, c, d = plane_params
    x, y, z = point
    return abs(a * x + b * y + c * z + d) / np.sqrt(a**2 + b**2 + c**2)

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

    def listener_callback(self, msg):
        depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth_image, dtype=np.float32)

        threshold_depth = 1.0  # Seuil de profondeur pour la détection du plan
        points = []

        for y in range(0, depth_image.shape[0], 10):  # Échantillonnage tous les 10 pixels
            for x in range(0, depth_image.shape[1], 10):
                depth = depth_image[y, x]
                if depth > 0 and depth < threshold_depth * 1000:  # Convertir en mm
                    points.append([x, y, depth])  # Ajouter le point (x, y, profondeur)

        # Convertir la liste de points en un tableau NumPy
        points = np.array(points)

        # Détection des plans avec RANSAC
        detected_planes = self.detect_planes(points)
        
        # Calculer la distance de chaque plan à la caméra
        for idx, plane_params in enumerate(detected_planes):
            distance = self.compute_distance_to_camera(plane_params)
            print(f"Plan {idx + 1} : Paramètres {plane_params}")
            print(f"Distance au plan {idx + 1} : {distance:.2f} mètres")
        
        # Affichage des plans détectés
        self.visualize_detected_planes(points, detected_planes)

    def detect_planes(self, points):
        ransac = RANSACRegressor(min_samples=3, residual_threshold=100, max_trials=50, random_state=42)
        detected_planes = []

        while len(points) >= 3:
            ransac.fit(points[:, :2], points[:, 2])  # Régression sur les points (x, y) pour prédire z

            inliers_mask = ransac.inlier_mask_
            inliers = points[inliers_mask]
            
            if len(inliers) >= 50:  # Filtrer les petits plans
                # Paramètres du plan a, b, c, d
                a, b = ransac.estimator_.coef_
                c = -1  # La régression nous donne z = ax + by + c, donc on fixe c = -1
                d = ransac.estimator_.intercept_
                detected_planes.append((a, b, c, d))  # Ajouter les paramètres du plan
                
            points = points[~inliers_mask]  # Enlever les points inliers déjà traités

        return detected_planes

    def compute_distance_to_camera(self, plane_params):
        # La position de la caméra est supposée être à l'origine (0, 0, 0)
        camera_position = np.array([0.0, 0.0, 0.0])
        distance = distance_to_plane(camera_position, plane_params)
        return distance

    def visualize_detected_planes(self, points, detected_planes):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Affichage des points détectés
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r', s=1)

        # Affichage des plans détectés
        for plane_params in detected_planes:
            a, b, c, d = plane_params

            # Créer une grille de points (x, y)
            x = np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 50)
            y = np.linspace(np.min(points[:, 1]), np.max(points[:, 1]), 50)
            X, Y = np.meshgrid(x, y)

            # Calculer les valeurs de Z sur le plan
            Z = (-a * X - b * Y - d) / c  # Equation du plan: ax + by + cz + d = 0

            # Affichage de la surface du plan
            ax.plot_surface(X, Y, Z, color='b', alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
