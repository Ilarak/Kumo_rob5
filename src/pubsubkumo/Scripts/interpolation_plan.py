#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from example_interfaces.msg import Int32
from sklearn.linear_model import LinearRegression
import cv2
import numpy as np

tresholdmin = 170;
tresholdmax = 400;

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
        # self.timer = self.create_timer(timer_period, self.timer_callback)

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

        # Récupération des dimensions de l'image
        height, width = depth_array.shape
        
        plane_params = self.detect_plane(points)
        
        if plane_params:
                print(f"Plan détecté avec les paramètres : {plane_params}")

                # Estimer la distance d'un point à ce plan (par exemple, au centre de l'image)
                x_center, y_center = depth_image.shape[1] // 2, depth_image.shape[0] // 2
                depth_center = self.bilinear_interpolation(depth_image, x_center, y_center)  # Interpolation bilinéaire

                point = [x_center, y_center, depth_center]
                distance = distance_to_plane(point, plane_params)
                print(f"Distance au plan au centre de l'image : {distance:.2f} millimètres")
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap_with_plan = self.draw_plane_on_colormap(depth_colormap, plane_params)
        cv2.imshow("Depth Image", depth_colormap_with_plan)
        cv2.waitKey(1)
    
    # def timer_callback(self):
        # msg = Int32()
        # msg.data = self.direction
        # self.publisher_.publish(msg)
        # self.get_logger().info(f"Publishing : {msg.data}")
    
    def detect_plane(self, points):
        # Ajustez le nombre de points et la tolérance selon vos besoins
        if len(points) < 3:
            return None
        
                # Convertir les points en un tableau numpy pour la régression
        points = np.array(points)
        X = points[:, :2]  # Coordonnées (x, y)
        Z = points[:, 2]   # Profondeur (z)

        # Ajuster un plan à l'aide de la régression linéaire (ax + by + c = z)
        model = LinearRegression()
        model.fit(X, Z)

        a, b = model.coef_  # Coefficients pour x et y
        c = model.intercept_  # L'ordonnée à l'origine
        # d = -c
        # Vérifier si un des paramètres est invalide
        # if np.isnan(a) or np.isnan(b) or np.isnan(c) or np.isnan(d) or np.isinf(a) or np.isinf(b) or np.isinf(c):
        #     self.get_logger().warning("Un ou plusieurs paramètres du plan sont invalides (NaN ou Inf).")
        #     return None  # Retourner None si un paramètre est invalide

        if c == 0:
            self.get_logger().warning("Le paramètre 'c' du plan est zéro, le plan est parallèle à l'axe Z.")
            return None  # Si c == 0, retourner None car nous ne pouvons pas définir un plan valide.

        # L'équation du plan est ax + by + cz + d = 0, donc nous avons a, b, c, et d
        d = -c  # Calcul de d en fonction de c, car la régression donne c = a*x + b*y + c pour z
        return (a, b, c, -1)

        # Utilisation de l'algorithme RANSAC pour ajuster un plan
        points = np.array(points)
        centroid = np.mean(points, axis=0)
        points_centered = points - centroid
        
        # Calcul de la matrice de covariance
        cov_matrix = np.cov(points_centered.T)
        
        # Calcul de la décomposition en valeurs propres (eigenvectors/eigenvalues)
        _, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Le vecteur propre associé à la plus petite valeur propre donne la normale du plan
        normal_vector = eigenvectors[:, 0]
        d = -np.dot(normal_vector, centroid)
        
        return (*normal_vector, d)
    
    def bilinear_interpolation(self, depth_map, x, y):
        x1, y1 = int(x), int(y)
        x2, y2 = min(x1 + 1, depth_map.shape[1] - 1), min(y1 + 1, depth_map.shape[0] - 1)

        Q11 = depth_map[y1, x1]
        Q12 = depth_map[y2, x1]
        Q21 = depth_map[y1, x2]
        Q22 = depth_map[y2, x2]
        
        return (Q11 * (x2 - x) * (y2 - y) +
                Q21 * (x - x1) * (y2 - y) +
                Q12 * (x2 - x) * (y - y1) +
                Q22 * (x - x1) * (y - y1))
    
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
            # On compare ici la profondeur calculée à la profondeur mesurée pour afficher une différence
                if z > 0 and z < 1000:  # Limiter la profondeur à une plage raisonnable (en mm)
                    color_intensity = min(int(z * 0.03), 255)  # Ajuster l'intensité selon la profondeur

                # Si vous voulez dessiner une ligne ou un point, vous pouvez faire quelque chose comme :
                    cv2.circle(depth_colormap, (x, y), 1, (0, 255, 0), -1)  # Dessiner un petit cercle vert pour le plan

        return depth_colormap   




# class Publisher(Node):
#     def _initpub_ (self):
#         super().__init__('publisher')
#         self.publisher_ = self.create_publisher(Int, 'direction', 10)
#         timer_period = 0.5
#         self.timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         msg = int()
#         msg.data = 1
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Publishing : {msg.data}")



def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)
    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
