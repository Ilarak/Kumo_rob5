import math
import numpy as np

#fonction pour calculer la cinematique inverse
#unité : mm 
class Move:
    def __init__(self) :
        #Distance entre les servomoteur
        self.L_theta = 129
        self.L_beta = 75.66
        self.L_alpha = 53.0

        #Distance entre le 1er sevomoteur et le centre
        self.L_X_centre = 0.120 # distance entre le centre et les pattes des cotés 
        self.L_X_cote = 0.08138 # distance entre le centre et les pattes du centre
        self.L_Y = 0.13538

        #Angles de l'araignée
        self.roll = 0
        self.pitch = 0

        #Position maximum du mouvement en Y
        self.y_max = 38

        #Position de la patte au sol
        self.Z_sol = 90

        #Incrémentation du Y et Z à chaque iteration
        self.incr_y = 5
        max_z = 20
        self.add_z = max_z*self.incr_y/(self.y_max)
        
        #Données pour la difference de rotation des premier moteurs
        self.L_start = 30.151
        self.angle_start_pos = 0.78 #angle bias
        self.angle_start_neg = - self.angle_start_pos
        self.x_start_pos = math.cos(self.angle_start_pos)*self.L_start
        self.y_start_pos = math.sin(self.angle_start_pos)*self.L_start
        self.x_start_neg = math.cos(self.angle_start_neg)*self.L_start
        self.y_start_neg = math.sin(self.angle_start_neg)*self.L_start

        self.pos_ang = np.zeros(18, float)
        
    def pos_patte1_from_center(self,X,Y,Z, pos_x, pos_y):
        return -X-self.x_start + pos_x, Y + pos_y + self.y_start ,-Z  
        
    def pos_patte25_from_center(X,Y,Z, pos):
        return X+pos, Y, Z
        
    def cine_inv(self,X,Y,Z,angle):
        alpha = math.tan(Y/X)
        X = X - self.L_alpha*math.cos(alpha)
        Y = Y - self.L_alpha*math.sin(alpha)
        L = math.sqrt(X*X + Y*Y)
        
        A = math.sqrt(Z*Z + L*L)
        theta = self.calcule_angle_triangle(self.L_beta, self.L_theta, A) - math.pi
        if(Z/L>1):
            asin = 0.5*math.pi
        elif(Z/L>1):
            asin = -0.5*math.pi
        else:
            asin = math.asin(Z/L)
        beta = self.calcule_angle_triangle(self.L_beta, A, self.L_theta) - asin
        return alpha-angle, beta, theta

    def calcule_angle_triangle(self,c1,c2,c3):
        frac = (c1*c1+c2*c2-c3*c3)/(2*c1*c2)
        if (frac > 1 ):
            frac = 1
        elif(frac < -1):
            frac = -1
        angle = math.acos(frac)
        return angle

    def mouvement(self, Y, Z, pair1_avance, commande_type):
        X = 190 - self.L_start
        if pair1_avance:
            Y += self.incr_y
            if Y >= self.y_max:
                pair1_avance = False
                Z = self.Z_sol
            elif Y > 0:
                Z += self.add_z
            else:
                Z -= self.add_z
            self._executer_commande(commande_type, X, Y, self.Z_sol, Z)
        else:
            Y -= self.incr_y
            if Y <= -self.y_max:
                pair1_avance = True
                Z = self.Z_sol
            elif Y < 0:
                Z += self.add_z
            else:
                Z -= self.add_z
            self._executer_commande(commande_type, X, Y, Z, self.Z_sol)
        return Y, Z, pair1_avance


    def _executer_commande(self, commande_type, X, Y, Z1, Z2):
        if commande_type == 'avancer':
            self.commande_avancer(X, Y, Z2, Z1)
        elif commande_type == "reculer":
            self.commande_avancer(X, Y, Z1, Z2)
        elif commande_type == 'tourner_gauche':
            self.commande_tourner(X, Y, Z1, Z2)
        else:
            self.commande_tourner(X, Y, Z2, Z1)

    def reculer(self, Y, Z, pair1_avance):
        return self.mouvement(Y, Z, pair1_avance, 'reculer')

    def avancer(self, Y, Z, pair1_avance):
        return self.mouvement(Y, Z, pair1_avance, 'avancer')

    def tourner_gauche(self, Y, Z, pair1_avance):
        return self.mouvement(Y, Z, pair1_avance, 'tourner_gauche')

    def tourner_droite(self, Y, Z, pair1_avance):
        return self.mouvement(Y, Z, pair1_avance, 'tourner_droite')

    def fct_roll(self):
        z = np.array([math.sin(self.roll)*self.L_X_centre,math.sin(self.roll)*self.L_X_cote], float)
        return z
    
    def fct_pitch(self):
        return self.L_Y*math.sin(self.pitch)

    def commande_avancer(self,X, Y, Z_pair1, Z_pair2):
        # calcul Z par rapport aux angles roll et pitch
        z_pitch = self.fct_pitch()*1000
        z_roll = self.fct_roll()*1000

        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(X, Y, Z_pair2 + z_roll[1] + z_pitch ,self.angle_start_neg)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(X, -Y, Z_pair1 + z_roll[0] ,0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(X, Y, Z_pair2 + z_roll[1] - z_pitch ,self.angle_start_pos)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(X, Y, Z_pair1 - z_roll[1] + z_pitch,self.angle_start_pos)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(X, -Y, Z_pair2 - z_roll[0],0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(X, Y, Z_pair1 - z_roll[1] - z_pitch,self.angle_start_neg)
    

    def commande_tourner(self,X, Y, Z_pair1, Z_pair2):
        z_pitch = self.fct_pitch()*1000
        z_roll = self.fct_roll()*1000
        
        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(X, Y, Z_pair2 + z_roll[1] + z_pitch, self.angle_start_neg)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(X, -Y, Z_pair1 + z_roll[0] ,0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(X, Y, Z_pair2 + z_roll[1] - z_pitch,self.angle_start_pos)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(X, -Y, Z_pair1 - z_roll[1] + z_pitch,self.angle_start_pos)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(X, Y, Z_pair2 - z_roll[0],0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(X, -Y, Z_pair1 - z_roll[1]- z_pitch,self.angle_start_neg)

    
    def stars(self):
        z_pitch = self.fct_pitch()*1000
        z_roll = self.fct_roll()*1000

        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(266.509, 0, z_roll[1] + z_pitch, 0)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(266.509, 0, z_roll[0], 0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(266.509, 0, z_roll[1] - z_pitch, 0)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(266.509, 0, z_roll[1] + z_pitch, 0)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(266.509, 0, z_roll[0], 0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(266.509, 0, z_roll[1] - z_pitch, 0)

    def debout(self):
        z_pitch = self.fct_pitch()*1000
        z_roll = self.fct_roll()*1000

        X = 190 - self.L_start

        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(X, 0, self.Z_sol + z_roll[1] + z_pitch, self.angle_start_neg)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(X, 0, self.Z_sol + z_roll[0], 0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(X, 0, self.Z_sol + z_roll[1] - z_pitch, self.angle_start_pos)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(X, 0, self.Z_sol - z_roll[1] + z_pitch, self.angle_start_pos)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(X, 0, self.Z_sol - z_roll[0], 0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(X, 0, self.Z_sol - z_roll[1] - z_pitch, self.angle_start_neg)
