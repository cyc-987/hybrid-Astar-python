import turtle
import reeds_shepp as rs
import random as rd
import numpy as np
from map import Map

rad2deg = np.rad2deg


# drawing n units (eg turtle.forward(n)) will draw n * SCALE pixels
SCALE = 20

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
    

def InitAndDrawMap(map: Map):

    # 地图数据 (二维数组)
    map_data = map.map 

    # 颜色映射
    color_map = {
        0: "white",
        1: "black",
    }

    cell_size = SCALE  # 单元格大小

    # 获取地图的行数和列数
    rows = map.height
    cols = map.width

    # 初始化 turtle
    screen = turtle.Screen()
    screen.setup(width=cols * cell_size + 50, height=rows * cell_size + 50) # 设置窗口大小，留出一些边距
    screen.tracer(0) # 关闭动画

    tesla = turtle.Turtle()
    tesla.speed(0)       # 设置绘制速度为最快
    tesla.penup()       # 抬起画笔，避免移动时画线
    tesla.hideturtle()  # 隐藏 turtle 图标

    # 循环遍历二维数组并绘制地图
    offsetx = - (cols * cell_size) / 2
    offsety = - (rows * cell_size) / 2 + cell_size
    for row_index in range(rows):
        for col_index in range(cols):
            value = map_data[row_index][col_index]
            color = color_map.get(value, "white") # 获取对应颜色，如果值不在 color_map 中则默认为白色

            # 计算单元格左下角坐标
            x = (col_index * cell_size) + offsetx
            y = (row_index * cell_size) + offsety

            # 移动到左上角
            tesla.goto(x, y)
            tesla.pendown()
            tesla.fillcolor(color)
            tesla.begin_fill()
            
            # 绘制正方形单元格 (正确方法)
            for _ in range(4):
                tesla.forward(cell_size)
                tesla.right(90)
                
            tesla.end_fill()
            tesla.penup()
    
    # 显示turtle，恢复绘图速度
    tesla.showturtle()
    screen.tracer(1)
    tesla.shape('arrow')
    tesla.resizemode('user')
    tesla.shapesize(1, 1)
    

    return tesla, - (cols) / 2, - (rows) / 2
