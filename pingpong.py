from arm import Arm
import numpy as np
import pyglet

def plot():
    arm = Arm()
    window = pyglet.window.Window()

    ball_image = pyglet.image.load('ball.jpg')
    ball = pyglet.sprite.Sprite(ball_image, x=15, y=15)
    ball.dx = 2
    ball.dy = 2
    paddle_radius = 15

    def intersect_paddle(joints):
        inside_x = ball.x + 30 > window.width/2 + joints[0][len(arm.Wangles)] - paddle_radius and ball.x < window.width/2 + joints[0][len(arm.Wangles)] + paddle_radius
        inside_y = (ball.y < joints[1][len(arm.Wangles)] and ball.y + 30 > joints[1][len(arm.Wangles)])
        return inside_x and inside_y

    @window.event
    def on_draw():
        window.clear()
        joints = arm.get_joints()
        if ball.x > window.width - 30 or ball.x < 0:
            ball.dx *= -1
        if (ball.y > window.height - 30 or ball.y < 0) or intersect_paddle(joints):
            ball.dy *= -1
        ball.x += ball.dx
        ball.y += ball.dy
        ball.draw()
        for j in range(len(arm.Wangles)):
            pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (window.width/2 + joints[0][j], joints[1][j], window.width/2 + joints[0][j + 1], joints[1][j + 1])))
        pyglet.graphics.draw(2, pyglet.gl.GL_LINES, ('v2i', (window.width/2 + joints[0][len(arm.Wangles)] - paddle_radius, joints[1][len(arm.Wangles)], window.width/2 + joints[0][len(arm.Wangles)] + paddle_radius, joints[1][len(arm.Wangles)])))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        arm.Wangles = arm.slsqp([x - window.width/2, y])

    pyglet.app.run()
 
plot()