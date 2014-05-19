from arm import Arm
import numpy as np
import scipy as sp
import pyglet

def plot():
    pinv_arms  = [Arm(np.array([300 / i] * i)) for i in [3]]   # Array of arms with varying numbers of links to be drawn at the same time.
    tran_arms  = [Arm(np.array([300 / i] * i)) for i in [3]]
    sls_arms = [Arm(np.array([300 / i] * i)) for i in []]

    # Make the pyglet window!
    window = pyglet.window.Window()
    fps_display = pyglet.clock.ClockDisplay()

    # Get the ball, as a sprite, which will bounce around inside our window.
    ball_image = pyglet.image.load('ball.png')
    ball = pyglet.sprite.Sprite(ball_image, x=15, y=15)
    ball.dx = 4
    ball.dy = 4

    # paddle_radius is the distance from the end effector (center of the paddle) to the edge of the paddle which can hit the ball.
    paddle_radius = 15

    # Returns true if the paddle is currently inside the ball.
    def intersect_paddle(joints):
        inside_x = ball.x + ball.width > window.width/2 + joints[0][len(tran_arms[0].Wangles)] - paddle_radius and \
                   ball.x < window.width/2 + joints[0][len(tran_arms[0].Wangles)] + paddle_radius

        inside_y = ball.y < joints[1][len(tran_arms[0].Wangles)] + window.height/2 and \
                   ball.y + ball.height > joints[1][len(tran_arms[0].Wangles)] + window.height/2

        return inside_x and inside_y

    def add_arms_to_batch(batch, arms):
        if (len(arms) > 0):
            joints = [arms[i].get_joints() for i in range(len(arms))]
            for i in range(len(arms)):
                for j in range(len(arms[i].Wangles)):
                    batch.add(2, pyglet.gl.GL_LINES, None, \
                        ('v2i', (window.width/2 + joints[i][0][j], window.height/2 + joints[i][1][j], window.width/2 + joints[i][0][j + 1], window.height/2 + joints[i][1][j + 1])), \
                        ('c4B', arms[i].colors))

            # Draw a paddle at the end of the first arm.
            batch.add(2, pyglet.gl.GL_LINES, None, \
                ('v2i', (window.width/2  + joints[0][0][len(arms[0].Wangles)] - paddle_radius, \
                         window.height/2 + joints[0][1][len(arms[0].Wangles)], \
                         window.width/2  + joints[0][0][len(arms[0].Wangles)] + paddle_radius, \
                         window.height/2 + joints[0][1][len(arms[0].Wangles)])))

    # This method is called on a loop by the pyglet backend. Every frame is redrawn by this function.
    @window.event
    def on_draw():
        window.clear()
        fps_display.draw()

        tran_joints = [tran_arms[i].get_joints() for i in range(len(tran_arms))]

        # Check if the ball has hit the edge of the window or the paddle.
        if ball.x > window.width - ball.width or ball.x < 0:
            ball.dx *= -1
        if (ball.y > window.height - ball.height or ball.y < 0) or intersect_paddle(tran_joints[0]):
            ball.dy *= -1

        # Update the ball location.
        ball.x += ball.dx       # ball.x = ball.x + ball.dx
        ball.y += ball.dy
        ball.draw()

        # For every arm, draw a line segment from between the joints of the arm, in proper color.
        # See http://www.pyglet.org/doc/programming_guide/drawing_primitives.html for minimal help.
        arm_batch = pyglet.graphics.Batch()
        
        add_arms_to_batch(arm_batch, pinv_arms)
        add_arms_to_batch(arm_batch, tran_arms)
        add_arms_to_batch(arm_batch, sls_arms)

        arm_batch.draw()

    # This method is called every time the mouse moves (shocker). We don't have to redraw the window,
    # but we do have to calculate our new working angles. This is where we decide which method to use
    # to solve the inverse kinematic equation. As an idea, we could simply make a `solve` method in Arm
    # which calls the right version (optionally specified in the constructor).
    @window.event
    def on_mouse_motion(x, y, dx, dy):
        for i in range(len(pinv_arms)):
            pinv_arms[i].pinv_jacobian([x - window.width/2, y - window.height/2])
        for i in range(len(tran_arms)):
            tran_arms[i].transpose_jacobian([x - window.width/2, y - window.height/2])
        for i in range(len(sls_arms)):
            sls_arms[i].slsqp([x - window.width/2, y - window.height/2])

    pyglet.app.run()
 
plot()