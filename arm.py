import numpy as np
import scipy as sp
import scipy.optimize

class Arm:
    def __init__(self, lengths = None, start_angles = None):
        if start_angles is None: start_angles = np.array([0, 0, -sp.pi/4])
        if lengths is None: lengths = np.array([200, 200, 200])
        self.lengths = lengths
        self.Wangles = start_angles

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

    def get_joints(self, thetas = None):
        if thetas is None: thetas = self.Wangles
        result = np.zeros(shape = (2, len(thetas) + 1))
        for joint in range(1, len(thetas) + 1):
            for i in range(joint):
                result[0][joint] += self.lengths[i] * np.cos(thetas[0:(i + 1)].sum())
                result[1][joint] += self.lengths[i] * np.sin(thetas[0:(i + 1)].sum())
        return result.astype('int')

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
    def slsqp(self, xy):
        def distance_func(thetas, *args):
            '''minimize this'''
            return np.sqrt(np.sum((thetas-self.Wangles)**2))

        def constraint(thetas, xy, *args):
            return self.hand_xy(thetas) - xy

        # func is the function to minimize.
        # x0 is the initial list of arguments.
        # f_eqcons is a function which returns a vector which is all 0 after a successful optimization.
        # args is a list of extra arguments to pass to the f_eqcons function.
        # disp sets the desired printing verbosity.
        # See http://docs.scipy.org/doc/scipy-0.13.0/reference/generated/scipy.optimize.fmin_slsqp.html
        return sp.optimize.fmin_slsqp(func = distance_func, x0 = self.Wangles, f_eqcons = constraint, args = [xy], disp = 0)

    # Solve for the new joint angles using the pseudo-inverse of the Jacobian.
    # This only works well when xy is sufficiently close to the current position!
    # Otherwise, this will require multiple iterations.
    def pinv_jacobian(self, xy):
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

# Run the pseudo inverse method over and over, until the error falls below the threshold,
# then print the required number of iterations.
def threshold_test(threshold = None, goal = None):
    arm = Arm()
    if goal is None: goal = np.array([2, 1.5])
    if threshold is None: threshold = .01
    arm.Wangles = arm.pinv_jacobian(goal)
    count = 1
    while (error_between(goal, arm.hand_xy()) > threshold and count < 50):
        count += 1
        arm.Wangles = arm.pinv_jacobian(goal)

    print ("Threshold: " + str(threshold) + " -> " + str(count) + " iterations.")

def threshold_test_runner():
    for i in [-2, -5, -10, -20, -50]:
        test(10 ** (i))

# simple_test()
# print
# threshold_test_runner()