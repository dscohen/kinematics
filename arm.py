import numpy as np
import scipy as sp

def armLink(self, lengths=None):
    def __init__(self):
        self.max_angles = np.array([math.pi, math.pi, math.pi/4])
        self.min_angles = np.array([0,0,-math.pi/4])
        if lengths is None:
            lengths = np.array([1,1,1])
        self.lengths = lengths
        self.Wangles = self.min_angles

    def hand_xy(self):
        x = self.lengths[0] * np.cos(self.Wangles[0]) + \
            self.lengths[1] * np.cos(self.Wangles[0] + self.Wangles[1]) + \
            self.lengths[2] * np.cos(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        y = self.lengths[0] * np.sin(self.Wangles[0]) + \
            self.lengths[1] * np.sin(self.Wangles[0] + self.Wangles[1]) + \
            self.lengths[2] * np.sin(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])
            
        return np.array([x,y])

    def jacobian(self):
        x1 = -1 * self.lengths[0] * sin(self.Wangles[0]) - \
                  self.lengths[1] * sin(self.Wangles[0] + self.Wangles[1]) - \
                  self.lengths[2] * sin(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        x2 = -1 * self.lengths[1] * sin(self.Wangles[0] + self.Wangles[1]) - \
                  self.lengths[2] * sin(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        x3 = -1 * self.lengths[2] * sin(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        y1 = self.lengths[0] * cos(self.Wangles[0]) + \
             self.lengths[1] * cos(self.Wangles[0] + self.Wangles[1]) + \
             self.lengths[2] * cos(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        y2 = self.lengths[1] * cos(self.Wangles[0] + self.Wangles[1]) + \
             self.lengths[2] * cos(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])
             
        y3 = self.lengths[2] * cos(self.Wangles[0] + self.Wangles[1] + self.Wangles[2])

        return np.matrix([[x1, x2, x3], [y1, y2, y3]])


    def distance_func(self):
        '''minimize this'''
        np.temp = (self.Wangles-self.min_angles)**2
        return np.sqrt(np.sum(temp))

    def kinematics(self, xy):
        result = sp.optimize.fmin_slsqp(distance_func, self.Wangles, hand_xy-xy,xy,0)





