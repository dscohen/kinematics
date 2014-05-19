import numpy as np
import scipy as sp
import scipy.optimize
import time
import random

class Arm:
    def __init__(self, lengths = None, start_angles = None):
        if lengths is None: lengths = np.array([200,150,100])
        if start_angles is None: start_angles = np.zeros(len(lengths))
        self.J = None
        self.lengths = lengths
        self.Wangles = start_angles
        # Array of random colors used to make each arm a different color.
        self.colors = (255 - random.randint(0, 255), 255 - random.randint(0, 255), 255 - random.randint(0, 255), 255) * 2

    # Get the position of the end effector for the given joint angles.
    # If no angles are given, this uses the current working angles of the joints.
    def hand_xy(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        x = 0
        y = 0
        # For loop trickery to allow an arbitrary number of joints.
        for i in range(len(thetas)):
            x += self.lengths[i] * np.cos(thetas[0:(i + 1)].sum())
            y += self.lengths[i] * np.sin(thetas[0:(i + 1)].sum())
            
        return np.array([x,y])

    # Return a list of xy pairs corresponding the the locations of every joint of the arm.
    # Note that we add an initial joint at (0, 0).
    def get_joints(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        result = np.zeros(shape = (2, len(thetas) + 1))
        for joint in range(1, len(thetas) + 1):
            for i in range(joint):
                result[0][joint] += self.lengths[i] * np.cos(thetas[0:(i + 1)].sum())
                result[1][joint] += self.lengths[i] * np.sin(thetas[0:(i + 1)].sum())
        return result.astype('int') # Return this as type int because pyglet requires it.

    """Calculate the jacobian of the function which determines the position of the hand.
    The Jacobian gives a linear approximation for the necessary change in joint angles, t,
    given a change in hand position, p.
    p ~= J*t
    So, we need to solve this linear equation for t. If there exactly 3 joints, we can use the
    explicit inverse of J. But, when J is nearly singular, the inverse has poor performance.
    So, we explore different options, like the pseudo-inverse and transpose."""
    def jacobian(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        result = np.zeros(shape = (2, len(thetas)))
        for i in range(len(thetas)):
            for j in range(i, len(thetas)):
                result[0][i] -= self.lengths[j] * np.sin(thetas[0:(j + 1)].sum())
                result[1][i] += self.lengths[j] * np.cos(thetas[0:(j + 1)].sum())
        
        return result

    # Use the SciPy method fmin_slsqp, instead of the Jacobian, to solve for the new joint angles.
    def slsqp(self, xy, arm = None):
        if arm is None: arm = self
        def distance_func(thetas, *args):
            '''minimize this'''
            return np.sqrt(np.sum((thetas-arm.Wangles)**2))

        def optimize(thetas, xy, *args):
            return arm.hand_xy(thetas) - xy

        # func is the function to minimize.
        # x0 is the initial list of arguments.
        # f_eqcons is a function which returns a vector which is all 0 after a successful optimization.
        # args is a list of extra arguments to pass to the f_eqcons function.
        # disp sets the desired printing verbosity.
        # See http://docs.scipy.org/doc/scipy-0.13.0/reference/generated/scipy.optimize.fmin_slsqp.html
        start_time = time.time()
        arm.Wangles = sp.optimize.fmin_slsqp(func = distance_func, x0 = arm.Wangles, f_eqcons = optimize, args = (xy[0], xy[1]), disp = 0)
        return time.time() - start_time

    # Solve for the new joint angles using the pseudo-inverse of the Jacobian.
    # This only works well when xy is sufficiently close to the current position!
    # Otherwise, this will require multiple iterations.
    def pinv_jacobian(self, xy, arm = None):
        if arm is None: arm = self
        j = arm.jacobian()
        self.J = j
        
        start_time = time.time()
        inv = np.linalg.pinv(j)
        dtheta = np.dot(inv, (xy - arm.hand_xy()))
        end_time = time.time()
        arm.Wangles = arm.Wangles + dtheta
        return end_time - start_time

    def transpose_jacobian(self, xy, arm = None):
        if arm is None: arm = self
        j = arm.jacobian()
        t = np.transpose(j)
        self.J = t

        start_time = time.time()
        e = xy - arm.hand_xy()
        dotted = j.dot(t).dot(e)
        a = np.inner(dotted, e) / np.inner(dotted, dotted)
        dtheta = a * np.dot(t, e)
        end_time = time.time()
        arm.Wangles = arm.Wangles + dtheta
        return end_time - start_time

    def conditions(self):
        condition = np.linalg.cond(self.J)
        return condition

def error_between(a, b):
    return np.sqrt(((a - b)**2).sum())

# calculates maximum distance that end effetor can reach based on arm lengths:
#takes in np.array of endpoint (x,y), returns np array
#r = x^2 + y^2
def max_length(arms, goal):
    r = np.sum(arms)
    length = np.sqrt(goal[0]**2 + goal[1]**2)
    if abs(length) <= r:
        return goal
    if (goal[0] == 0) or (goal[1] == 0):
        return length * (goal/goal)
    m = float(goal[1])/goal[0]
    coeff = [(m**2+1),0,-1*r**2]
    x = np.roots(coeff)
    if (x[0] < 0 and goal[0] < 0):
        x = x[0]
    else:
        x = x[1]
    y = m*x
    return np.array([x,y])


# Run the solving method over and over, until the error falls below the threshold,
# then print the required number of iterations and time taken to reach that threshold.
def threshold_test(method_name = None, arm = None, threshold = None, goal = None):
    if arm is None: arm = Arm(np.array([100, 100]), np.array([sp.pi/4, sp.pi/4]))
    if method_name is None or method_name == "transpose": method = arm.transpose_jacobian
    elif method_name == "sls":       method = arm.slsqp
    elif method_name == "pinv":      method = arm.pinv_jacobian
    elif method_name == "transpose": method = arm.transpose_jacobian
    else:                            method = arm.transpose_jacobian
    if goal is None: goal = np.array([100, 0])
    if threshold is None: threshold = .01
    start = arm.hand_xy()
    time = method(goal)
    if method_name != "sls":
        c_num = arm.conditions()
    count = 1
    while (error_between(goal, arm.hand_xy()) > threshold and count < 50):
        count += 1
        time += method(goal, arm = arm)
        if method_name != "sls":
            temp = arm.conditions()
            if (c_num > temp):
                c_num = temp

    print (str(start) + " -> " + str(goal) + " (" + str(error_between(start, goal)) + ")")
    print ("Threshold: " + str(threshold) + " -> " + str(count) + " iterations (" + str(time * 1000) + " ms, " + method_name + ")")
    # if (method_name != "sls"):
    #     print ("Condition number -> " + str(c_num))
    print ("")


def threshold_test_runner():
    for i in [-1, -3, -5, -10, -15]:
        threshold_test(method_name = "sls", threshold = 10 ** (i))
        threshold_test(method_name = "pinv", threshold = 10 ** (i))
        threshold_test(method_name = "transpose", threshold = 10 ** (i))

def goal_test_runner():
    for item in [[200,0],[180,0],[0,0],[-70,-170]]:
        threshold_test(method_name = "sls", threshold = 10 ** (-5), goal = np.array([item[0],item[1]]))
        threshold_test(method_name = "pinv", threshold = 10 ** (-5), goal = np.array([item[0],item[1]]))
        threshold_test(method_name = "transpose", threshold = 10 ** (-5), goal = np.array([item[0],item[1]]))

print "Threshold test"
threshold_test_runner()
# print "Goal test"
# goal_test_runner()
