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

    def jacobian(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        result = np.zeros(shape = (2, len(thetas)))
        for i in range(len(thetas)):
            for j in range(i, len(thetas)):
                result[0][i] -= self.lengths[j] * np.sin(thetas[0:(j + 1)].sum())
                result[1][i] += self.lengths[j] * np.cos(thetas[0:(j + 1)].sum())
        
        return result

    def slsqp(self, xy):
        def distance_func(Wangles, *args):
            '''minimize this'''
            return np.sqrt(np.sum((Wangles-self.min_angles)**2))

        def constraint(thetas, xy, *args):
            return self.hand_xy(thetas) - xy

        return sp.optimize.fmin_slsqp(func = distance_func, x0 = self.Wangles, f_eqcons = constraint, args = [xy], disp = 0)

    def pinv_jacobian(self, xy):    # Only works well when xy is close to the current position! Otherwise, this is pretty bad.
        inv = np.linalg.pinv(self.jacobian())
        return self.Wangles + np.dot(inv, (xy - self.hand_xy()))

def error_between(a, b):
    return np.sqrt(((a - b)**2).sum())

def simple_test():
    arm = Arm()
    goal = np.array([2.65, -.6])
    print ("Start:       " + str(arm.hand_xy()))
    print ("Goal:        " + str(goal))
    print
    pinv_end = arm.hand_xy(arm.pinv_jacobian(goal))
    print ("Pinv end:    " + str(pinv_end))

    arm.Wangles = arm.slsqp(goal)
    end = arm.hand_xy()

    print ("Scipy end:   " + str(end))
    print
    print ("Scipy Error: " + str(error_between(goal, end)))
    print ("Pinv  Error: " + str(error_between(goal, pinv_end)))
simple_test()