import numpy as np
import math
import turtle

from map import Map
from basic_hybridAstar import HybridAStar, Object
import draw

pi = math.pi
sin = np.sin
cos = np.cos

def main():
    # 创建地图
    m = Map(map_type=2)
    m.MapInfo()
    tesla, offsetx, offsety = draw.InitAndDrawMap(m)
    car = Object()
    
    # 设置起点和终点
    offset = (offsetx, offsety, 0)
    start = (10, 15, 0)
    goal = (25, 7, pi)
    print(f"Start: {start}, Goal: {goal}")
    draw.goto(tesla, (start[0]+offset[0], start[1]+offset[1], start[2]))
    draw.vec(tesla)
    draw.goto(tesla, (goal[0]+offset[0], goal[1]+offset[1], goal[2]))
    draw.vec(tesla)
    
    # 启动算法
    ha = HybridAStar(start, goal, m, car)
    path, rs_path = ha.Perform()
    
    # 打印结果
    print("Path:")
    print("Nodes:")
    tesla.pencolor(0,1,0)
    for p in path:
        print(f"({p[0]}, {p[1], p[2]})")
        lastnode = (p[0], p[1], p[2])
        draw.goto(tesla, (p[0]+offset[0], p[1]+offset[1], p[2]))
        draw.vec(tesla)
    print("RS Path:")
    for elements in rs_path:
        print(f"({elements.steering}, {elements.gear}, {elements.value:.2f})")
    tesla.pencolor(0,1,1)
    tesla.pensize(3)
    draw.goto(tesla, (lastnode[0]+offset[0], lastnode[1]+offset[1], lastnode[2]))
    draw.draw_path(tesla, rs_path, car.radius)
    
    # 平滑路径
    rs_path_smooth = ha.SmoothPath(path)
    if rs_path_smooth:
        tesla.pencolor(1,0.5,0)
        tesla.pensize(5)
        draw.goto(tesla, (start[0]+offset[0], start[1]+offset[1], start[2]))
        tesla.speed(3)
        for rs_path_tmp in rs_path_smooth:
            draw.draw_path(tesla, rs_path_tmp, car.radius)
        draw.draw_path(tesla, rs_path, car.radius)

    turtle.done()
    
if __name__ == '__main__':
    main()