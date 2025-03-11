import turtle
import reeds_shepp as rs
import random as rd
import numpy as np

rad2deg = np.rad2deg


# drawing n units (eg turtle.forward(n)) will draw n * SCALE pixels
SCALE = 40

def scale(x):
    """
    Scale the input coordinate(s).
    """
    if type(x) is tuple or type(x) is list:
        return [p * SCALE for p in x]
    return x * SCALE

def unscale(x):
    """
    Unscale the input coordinate(s).
    """
    if type(x) is tuple or type(x) is list:
        return [p / SCALE for p in x]
    return x / SCALE

# note: bob is a turtle

def vec(bob):
    """
    Draw an arrow.
    """
    bob.down()
    bob.pensize(3)
    bob.forward(scale(1.2))
    bob.right(25)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.left(50)
    bob.backward(scale(.4))
    bob.forward(scale(.4))
    bob.right(25)
    bob.pensize(1)
    bob.up()

def goto(bob, pos, scale_pos=True):
    """
    Go to a position without drawing.
    """
    bob.up()
    if scale_pos:
        bob.setpos(scale(pos[:2]))
    else:
        bob.setpos(pos[:2])
    bob.setheading(rad2deg(pos[2]))
    bob.down()

def draw_path(bob, path, r):
    """
    Draw the path (list of rs.PathElements).
    """
    for e in path:
        gear = e.gear
        if e.steering == -1:
            bob.circle(r * scale(1), gear * rad2deg(e.value))
        elif e.steering == 1:
            bob.circle(- r * scale(1), gear * rad2deg(e.value))
        elif e.steering == 0:
            bob.forward(gear * r * scale(e.value))

def set_random_pencolor(bob):
    """
    Draws noodles.
    """
    r, g, b = 1, 1, 1
    while r + g + b > 2.5:
        r, g, b = rd.uniform(0, 1), rd.uniform(0, 1), rd.uniform(0, 1)
    bob.pencolor(r, g, b)
