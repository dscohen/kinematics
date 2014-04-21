from arm import Arm
import scipy as sp
import numpy as np
import pyglet

def plot():
    window = pyglet.window.Window()
    arm = Arm(lengths = np.array([200, 150, 100, 75]), start_angles = np.array([0, 0, 0, 0]))

    joints = arm.get_joints()

    print joints

    @window.event
    def on_draw():
        window.clear()
        joints = arm.get_joints()
        for j in range(len(arm.Wangles)):
             pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (window.width/2 + joints[0][j], joints[1][j], window.width/2 + joints[0][j + 1], joints[1][j + 1])))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        arm.Wangles = arm.slsqp([x - window.width/2, y])

    pyglet.app.run()
 
plot()