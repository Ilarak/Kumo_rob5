import math
import numpy as np
import time

#fonction pour calculer la cinematique inverse
#unité : mm 
class Move:
    def __init__(self) :
        self.L_theta = 129
        self.L_beta = 75.66
        self.L_alpha = 53.0
        self.incr_y = 5
        self.roll = 0
        self.pitch = 0
        self.y_max = 38
        self.Z_sol = 90
        max_z = 20
        self.add_z = max_z*self.incr_y/(self.y_max)
        self.test = True
        
        #les données pour la difference de rotation des premier moteurs
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
        # on commance à alpha avec deja bien droit
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
    
    def test_(self):
        X = 190 - self.L_start
        self.commande_avancer_cine_inv(X,0,100,100)
        

    def reculer(self,Y,Z, pair1_avance):
        X = 190 - self.L_start
        if(pair1_avance):
            Y+=self.incr_y
            if (Y>=self.y_max):
                pair1_avance = False
                Z=self.Z_sol
            elif (Y > 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_avancer_cine_inv(X,Y,self.Z_sol, Z)
        else:
            Y-=self.incr_y
            if(Y<=-self.y_max):
                pair1_avance = True
                Z=self.Z_sol
            elif (Y < 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_avancer_cine_inv(X,Y,Z, self.Z_sol)
        return Y, Z, pair1_avance
    
    def avancer(self,Y,Z, pair1_avance):
        X = 190 - self.L_start
        if(pair1_avance):
            Y+=self.incr_y
            if (Y>=self.y_max):
                pair1_avance = False
                Z = self.Z_sol
            elif (Y> 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_avancer_cine_inv(X,Y,Z,self.Z_sol)
        else:
            Y-=self.incr_y
            if(Y<=-self.y_max):
                pair1_avance = True
                Z = self.Z_sol
            elif (Y < 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_avancer_cine_inv(X,Y,self.Z_sol,Z)
        return Y, Z, pair1_avance
    
    def tourner_gauche(self,Y,Z, pair1_avance):
        X = 190 - self.L_start
        if(pair1_avance):
            Y+=self.incr_y
            if (Y>=self.y_max):
                pair1_avance = False
                Z = self.Z_sol
            elif (Y> 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_tourner(X,Y,self.Z_sol, Z)
        else:
            Y-=self.incr_y
            if(Y<=-self.y_max):
                pair1_avance = True
                Z = self.Z_sol
            elif (Y < 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_tourner(X,Y,Z, self.Z_sol)
        return Y, Z, pair1_avance
    
    def tourner_droite(self,Y,Z, pair1_avance):
        X = 190 - self.L_start
        if(pair1_avance):
            Y+=self.incr_y
            if (Y>=self.y_max):
                pair1_avance = False
                Z = self.Z_sol
            elif (Y> 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_tourner(X,Y,Z, self.Z_sol)
        else:
            Y-=self.incr_y
            if(Y<=-self.y_max):
                pair1_avance = True
                Z = self.Z_sol
            elif (Y < 0):
                Z+=self.add_z
            else:
                Z-=self.add_z
            self.commande_tourner(X,Y,self.Z_sol, Z)
        return Y, Z, pair1_avance

    def fct_roll(self):
        z = np.array([math.sin(self.roll)*0.120,math.sin(self.roll)*0.08138], float)
        return z
    def fct_pitch(self):
        return 0.13538*math.sin(self.pitch)

    def commande_avancer_cine_inv(self,X, Y, Z_pair1, Z_pair2):
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
        
        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(X, Y, Z_pair2,self.angle_start_neg)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(X, -Y, Z_pair1,0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(X, Y, Z_pair2,self.angle_start_pos)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(X, -Y, Z_pair1,self.angle_start_pos)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(X, Y, Z_pair2,0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(X, -Y, Z_pair1,self.angle_start_neg)


    def check_collision():
        return False
    
    def stars(self):
        self.pos_ang[0],self.pos_ang[1],self.pos_ang[2] = self.cine_inv(266.509, 0, 0, 0)
        self.pos_ang[3],self.pos_ang[4],self.pos_ang[5] = self.cine_inv(266.509, 0, 0, 0)
        self.pos_ang[6],self.pos_ang[7],self.pos_ang[8] = self.cine_inv(266.509, 0, 0, 0)
        self.pos_ang[9],self.pos_ang[10],self.pos_ang[11] = self.cine_inv(266.509, 0, 0, 0)
        self.pos_ang[12],self.pos_ang[13],self.pos_ang[14] = self.cine_inv(266.509, 0, 0, 0)
        self.pos_ang[15],self.pos_ang[16],self.pos_ang[17] = self.cine_inv(266.509, 0, 0, 0)
