#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.msg import Int32
from sklearn.linear_model import RANSACRegressor
from sklearn.cluster import DBSCAN
import cv2
import numpy as np

tresholdmin = 170
tresholdmax = 400

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
            10
        )
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

        cv2.imshow("Depth Image (Raw)", depth_image)
        cv2.waitKey(1)

        threshold_depth = 1.0  # Seuil de profondeur pour la détection du plan
        points = []

        # Échantillonnage des points dans l'image
        for y in range(0, depth_image.shape[0], 10):  # Échantillonnage tous les 10 pixels
            for x in range(0, depth_image.shape[1], 10):
                depth = depth_image[y, x]
                if depth > 0 and depth < threshold_depth * 1000:  # Convertir en mm
                    points.append([x, y, depth])  # Ajouter le point (x, y, profondeur)

        points = np.array(points)

        # Afficher les points extraits pour vérification
        print(f"Nombre de points extraits : {len(points)}")
        if len(points) > 0:
            print(f"Exemple de points : {points[:5]}")
        
        # Clustering des points pour segmenter l'image en régions potentielles pour chaque plan
        clustering = DBSCAN(eps=30, min_samples=25).fit(points[:, :2])  # Clustering sur les coordonnées x, y
        labels = clustering.labels_

        unique_labels = set(labels)
        print(f"Nombre de clusters détectés : {len(unique_labels)}")

        # Application de RANSAC pour chaque cluster détecté
        #print(f"prelabel {labels}")
        for label in unique_labels:
            if label == -1:
                continue  # Ignore les points considérés comme du bruit
            #print(f"oui")
            cluster_points = points[labels == label]

            # Détection du plan pour ce cluster à l'aide de RANSAC
            plane_params = self.detect_plane_ransac(cluster_points)
            if plane_params:
                print(f"Plan détecté pour le cluster {label} avec les paramètres : {plane_params}")

                # Visualisation du plan détecté dans l'image
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap_with_plan = self.draw_plane_on_colormap(depth_colormap, plane_params)
                cv2.imshow(f"Plan détecté {label}", depth_colormap_with_plan)
            #cv2.imshow("Plan détecté", depth_colormap)
            cv2.waitKey(1)
            #cv2.destroyAllWindows()

    def detect_plane_ransac(self, points):
        # Convertir les points en un tableau numpy
        X = points[:, :2]  # Coordonnées (x, y)
        Z = points[:, 2]   # Profondeur (z)

        # Ajuster le modèle RANSAC
        ransac = RANSACRegressor()
        ransac.fit(X, Z)
        
        if np.any(np.isnan(ransac.estimator_.coef_)) or np.any(np.isnan(ransac.estimator_.intercept_)):
            return None
        
        # Récupérer les coefficients du plan
        a, b = ransac.estimator_.coef_
        c = ransac.estimator_.intercept_
        d = -c

        return a, b, c, d

    def draw_plane_on_colormap(self, depth_colormap, plane_params):
        if plane_params is None:
            return depth_colormap  # Si aucun plan n'a été détecté, retourner simplement la carte de profondeur sans modifications.

        # Récupérer les paramètres du plan
        a, b, c, d = plane_params
        height, width = depth_colormap.shape[:2]

        # Pour chaque pixel de l'image, calculer la profondeur du plan à cette position
        for y in range(height):
            for x in range(width):
                # Calculer la profondeur z du plan pour le pixel (x, y)
                z = -(a * x + b * y + d) / c

                # Si la profondeur calculée est valide (dans un intervalle raisonnable), dessiner sur l'image
                if z > 0 and z < 1000:  # Limiter la profondeur à une plage raisonnable (en mm)
                    color_intensity = min(int(z * 0.03), 255)  # Ajuster l'intensité selon la profondeur

                    # Dessiner un petit cercle vert pour le plan
                    cv2.circle(depth_colormap, (x, y), 1, (0, 255, 0), -1)

        return depth_colormap

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
