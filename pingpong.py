from arm import Arm
import numpy as np
import scipy as sp
import pyglet
import random

def plot():
    arms = [Arm(np.array([400 / i] * i)) for i in [20, 7, 3]]   # Array of arms with varying numbers of links to be drawn at the same time.

    # Array of random colors used to make each arm a different color.
    colors = [(255 - random.randint(0, 255), 255 - random.randint(0, 255), 255 - random.randint(0, 255), 255) * 2 for i in range(len(arms))]

    # Make the pyglet window!
    window = pyglet.window.Window()

    # Get the ball, as a sprite, which will bounce around inside our window.
    ball_image = pyglet.image.load('ball.png')
    ball = pyglet.sprite.Sprite(ball_image, x=15, y=15)
    ball.dx = 4
    ball.dy = 4

    # paddle_radius is the distance from the end effector (center of the paddle) to the edge of the paddle which can hit the ball.
    paddle_radius = 15

    # Returns true if the paddle is currently inside the ball.
    def intersect_paddle(joints):
        inside_x = ball.x + ball.width > window.width/2 + joints[0][len(arms[0].Wangles)] - paddle_radius and \
                   ball.x < window.width/2 + joints[0][len(arms[0].Wangles)] + paddle_radius

        inside_y = ball.y < joints[1][len(arms[0].Wangles)] and \
                   ball.y + ball.height > joints[1][len(arms[0].Wangles)]

        return inside_x and inside_y

    # This method is called on a loop by the pyglet backend. Every frame is redrawn by this function.
    @window.event
    def on_draw():
        window.clear()

        # Get the list of joint positions, for every arm.
        joints = [arms[i].get_joints() for i in range(len(arms))]

        # Check if the ball has hit the edge of the window or the paddle.
        if ball.x > window.width - ball.width or ball.x < 0:
            ball.dx *= -1
        if (ball.y > window.height - ball.height or ball.y < 0) or intersect_paddle(joints[0]):
            ball.dy *= -1

        # Update the ball location.
        ball.x += ball.dx
        ball.y += ball.dy
        ball.draw()

        # For every arm, draw a line segment from between the joints of the arm, in proper color.
        # See http://www.pyglet.org/doc/programming_guide/drawing_primitives.html for minimal help.
        for i in range(len(arms)):
            for j in range(len(arms[i].Wangles)):
                pyglet.graphics.draw(2, pyglet.gl.GL_LINES, \
                    ('v2i', (window.width/2 + joints[i][0][j], joints[i][1][j], window.width/2 + joints[i][0][j + 1], joints[i][1][j + 1])), \
                    ('c4B', colors[i]))

        # Draw a paddle at the end of the first arm.
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES, \
            ('v2i', (window.width/2 + joints[0][0][len(arms[0].Wangles)] - paddle_radius, \
                    joints[0][1][len(arms[0].Wangles)], \
                    window.width/2 + joints[0][0][len(arms[0].Wangles)] + paddle_radius, \
                    joints[0][1][len(arms[0].Wangles)])))

    # This method is called every time the mouse moves (shocker). We don't have to redraw the window,
    # but we do have to calculate our new working angles. This is where we decide which method to use
    # to solve the inverse kinematic equation. As an idea, we could simply make a `solve` method in Arm
    # which calls the right version (optionally specified in the constructor).
    @window.event
    def on_mouse_motion(x, y, dx, dy):
        for i in range(len(arms)):
            arms[i].Wangles = arms[i].pinv_jacobian([x - window.width/2, y])
            # arm.Wangles = arm.slsqp([x - window.width/2, y])

    pyglet.app.run()
 
plot()