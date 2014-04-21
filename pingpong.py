from arm import Arm
import pyglet

def plot():
    arm = Arm()
    window = pyglet.window.Window()

    joints = arm.get_joints()

    @window.event
    def on_draw():
        window.clear()
        joints = arm.get_joints()
        print range(len(arm.Wangles))
        for j in range(len(arm.Wangles)):
             pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (window.width/2 + joints[0][j], joints[1][j], window.width/2 + joints[0][j + 1], joints[1][j + 1])))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        arm.Wangles = arm.slsqp([x - window.width/2, y]) # get new arm angles
        print (str(x) + ", " + str(y))
        print arm.hand_xy()

    pyglet.app.run()
 
plot()