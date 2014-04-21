import numpy as np
import scipy as sp
import scipy.optimize

class Arm:
    def __init__(self, lengths = None):
        self.max_angles = np.array([sp.pi, sp.pi, sp.pi/4])
        self.min_angles = np.array([0,0,-sp.pi/4])
        if lengths is None:
            lengths = np.array([1,1,1])
        self.lengths = lengths
        self.Wangles = self.min_angles

    def hand_xy(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        x = 0
        y = 0
        for i in range(len(thetas)):
            x += self.lengths[i] * np.cos(thetas[0:(i + 1)].sum())
            y += self.lengths[i] * np.sin(thetas[0:(i + 1)].sum())
            
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

    def slsqp(self, xy):
        def distance_func(Wangles, *args):
            '''minimize this'''
            return np.sqrt(np.sum((Wangles-self.min_angles)**2))

        def constraint(thetas, xy, *args):
            return self.hand_xy(thetas) - xy

        return sp.optimize.fmin_slsqp(func = distance_func, x0 = self.Wangles, f_eqcons = constraint, args = [xy], disp = 0)

def error_between(a, b):
    return np.sqrt(((a - b)**2).sum())

def simple_test():
    arm = Arm()
    goal = np.array([.5, 1])
    arm.Wangles = arm.slsqp(goal)
    end = arm.hand_xy()
    print error_between(goal, end)

simple_test()