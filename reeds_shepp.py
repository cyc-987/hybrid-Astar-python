import numpy as np
import math
import turtle
import draw

sin = np.sin
cos = np.cos
pi = math.pi

class APathComponent:
    def __init__(self,
                 value: float,
                 steering: int,
                 gear: int,):
        '''RS曲线的基本组件
        Args:
            value(float): 值
            steering(int): 转向角（-1左, 0直行, 1右）
            gear(int): 档位（-1后退, 1前进）
        '''
        self.value = value
        self.steering = steering
        self.gear = gear
        
        if value < 0:
            self.value = -value
            self.gear = -gear
    
    def __repr__(self):
        return f"({self.value:.2f}, {self.steering}, {self.gear})"

    
class RSCurve:
    def __init__(self, 
                 x0: float, y0: float, theta0: float, 
                 R: float):
        '''计算RS曲线
        Args:
            x0(float): 起点x坐标
            y0(float): 起点y坐标
            theta0(float): 起点角度
            R(float,>0): 曲率半径
        '''
        self.x0 = x0
        self.y0 = y0
        self.theta0 = theta0
        self.R = R
    
    def __call__(self, x1: float, y1: float, theta1: float):
        '''计算并返回最短RS曲线
        Args:
            x1(float): 终点x坐标
            y1(float): 终点y坐标
            theta1(float): 终点角度
        Returns:
            path(list): RS曲线
        '''
        x1_n, y1_n, theta1_n = self._Normalization(x1, y1, theta1)
        paths = self._Generate(x1_n, y1_n, theta1_n)
        # 寻找最短路径
        min_path = min(paths, key=lambda path: sum([e.value for e in path]))
        return min_path, paths
        
    def _Generate(self, x1, y1, theta1):
        '''生成RS曲线'''
        path_generate_functions = [path1, path2, path3, path4, path5, path6, \
                                   path7, path8, path9, path10, path11, path12]
        paths = []
        # 生成十二种基本曲线
        for a_path in path_generate_functions:
            # 每条曲线生成四种变体
            paths.append(a_path(x1, y1, theta1))
            paths.append(reflect(a_path(x1, -y1, -theta1)))
            paths.append(timeflip(a_path(-x1, y1, -theta1)))
            paths.append(reflect(timeflip(a_path(-x1, -y1, theta1))))
        # 去掉空路径
        paths = list(filter(None, paths))
        # 单个路径中去掉0
        for i in range(len(paths)):
            paths[i] = list(filter(lambda x: x.value != 0, paths[i]))
        
        return paths

    def _Normalization(self, x1, y1, theta1):
        '''归一化坐标系
        Args:
            x1(float): 终点x坐标
            y1(float): 终点y坐标
            theta1(float): 终点角度
        Returns:
            new_x, new_y, new_theta(float): 归一化后的坐标
        '''
        dx = x1 - self.x0
        dy = y1 - self.y0
        new_x = dx * cos(self.theta0) + dy * sin(self.theta0)
        new_y = -dx * sin(self.theta0) + dy * cos(self.theta0)
        new_x /= self.R
        new_y /= self.R
        new_theta = theta1 - self.theta0 
        return new_x, new_y, new_theta
    
def PathLen(path, r):
    '''计算路径长度'''
    return sum([e.value for e in path]) * r

'''The code below is modified from ref/reeds-shepp-curves/reeds_shepp.py'''

def M(theta):
    """
    Return the angle phi = theta mod (2 pi) such that -pi <= theta < pi.
    """
    theta = theta % (2*math.pi)
    if theta < -math.pi: return theta + 2*math.pi
    if theta >= math.pi: return theta - 2*math.pi
    return theta

def R(x, y):
    """
    Return the polar coordinates (r, theta) of the point (x, y).
    """
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    return r, theta

def timeflip(path):
    """
    timeflip transform described around the end of the article
    """
    new_path = [APathComponent(e.value, e.steering, -e.gear) for e in path]
    return new_path

def reflect(path):
    """
    reflect transform described around the end of the article
    """
    new_path = [APathComponent(e.value, -e.steering, e.gear) for e in path]
    return new_path

def path1(x, y, phi):
    """
    Formula 8.1: CSC (same turns)
    """
    path = []

    u, t = R(x - math.sin(phi), y - 1 + math.cos(phi))
    v = M(phi - t)

    path.append(APathComponent(t, -1, 1))
    path.append(APathComponent(u, 0, 1))
    path.append(APathComponent(v, -1, 1))

    return path


def path2(x, y, phi):
    """
    Formula 8.2: CSC (opposite turns)
    """
    path = []

    rho, t1 = R(x + math.sin(phi), y - 1 - math.cos(phi))

    if rho * rho >= 4:
        u = math.sqrt(rho * rho - 4)
        t = M(t1 + math.atan2(2, u))
        v = M(t - phi)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 0, 1))
        path.append(APathComponent(v, 1, 1))

    return path


def path3(x, y, phi):
    """
    Formula 8.3: C|C|C
    """
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(phi - t - u)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 1, -1))
        path.append(APathComponent(v, -1, 1))

    return path


def path4(x, y, phi):
    """
    Formula 8.4 (1): C|CC
    """
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(t + u - phi)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 1, -1))
        path.append(APathComponent(v, -1, -1))

    return path


def path5(x, y, phi):
    """
    Formula 8.4 (2): CC|C
    """
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        u = math.acos(1 - rho*rho/8)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 - A)
        v = M(t - u - phi)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 1, 1))
        path.append(APathComponent(v, -1, -1))

    return path


def path6(x, y, phi):
    """
    Formula 8.7: CCu|CuC
    """
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        if rho <= 2:
            A = math.acos((rho + 2) / 4)
            t = M(theta + math.pi/2 + A)
            u = M(A)
            v = M(phi - t + 2*u)
        else:
            A = math.acos((rho - 2) / 4)
            t = M(theta + math.pi/2 - A)
            u = M(math.pi - A)
            v = M(phi - t + 2*u)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 1, 1))
        path.append(APathComponent(u, -1, -1))
        path.append(APathComponent(v, 1, -1))

    return path


def path7(x, y, phi):
    """
    Formula 8.8: C|CuCu|C
    """
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)
    u1 = (20 - rho*rho) / 16

    if rho <= 6 and 0 <= u1 <= 1:
        u = math.acos(u1)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 1, -1))
        path.append(APathComponent(u, -1, -1))
        path.append(APathComponent(v, 1, 1))

    return path


def path8(x, y, phi):
    """
    Formula 8.9 (1): C|C[pi/2]SC
    """
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(2, u+2)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi + math.pi/2)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(math.pi/2, 1, -1))
        path.append(APathComponent(u, 0, -1))
        path.append(APathComponent(v, -1, -1))

    return path


def path9(x, y, phi):
    """
    Formula 8.9 (2): CSC[pi/2]|C
    """
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(u+2, 2)
        t = M(theta + math.pi/2 - A)
        v = M(t - phi - math.pi/2)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 0, 1))
        path.append(APathComponent(math.pi/2, 1, 1))
        path.append(APathComponent(v, -1, -1))

    return path


def path10(x, y, phi):
    """
    Formula 8.10 (1): C|C[pi/2]SC
    """
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta + math.pi/2)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(math.pi/2, 1, -1))
        path.append(APathComponent(u, 0, -1))
        path.append(APathComponent(v, 1, -1))

    return path


def path11(x, y, phi):
    """
    Formula 8.10 (2): CSC[pi/2]|C
    """
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(u, 0, 1))
        path.append(APathComponent(math.pi/2, -1, 1))
        path.append(APathComponent(v, 1, -1))

    return path


def path12(x, y, phi):
    """
    Formula 8.11: C|C[pi/2]SC[pi/2]|C
    """
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 4:
        u = math.sqrt(rho*rho - 4) - 4
        A = math.atan2(2, u+4)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(APathComponent(t, -1, 1))
        path.append(APathComponent(math.pi/2, 1, -1))
        path.append(APathComponent(u, 0, -1))
        path.append(APathComponent(math.pi/2, -1, -1))
        path.append(APathComponent(v, 1, 1))

    return path

if __name__ == '__main__':
    # init turtle
    tesla = turtle.Turtle()
    tesla.speed(0) # 0: fast; 1: slow, 8.4: cool
    tesla.shape('arrow')
    tesla.resizemode('user')
    tesla.shapesize(1, 1)
    
    tesla.speed(0)
    
    start = (-2,-3,pi/3)
    end = (7,4,pi/2)
    r = 2
    
    draw.goto(tesla, start)
    draw.vec(tesla)
    draw.goto(tesla, end)
    draw.vec(tesla)
    
    
    min_path, paths = RSCurve(*start, r)(*end)
    
    # tesla.speed(50)
    # for path in paths:
    #     draw.set_random_pencolor(tesla)
    #     draw.goto(tesla, start)
    #     draw.draw_path(tesla, path,r)
    
    draw.goto(tesla, start)
    tesla.speed(1)
    tesla.pencolor(0,1,1)
    tesla.pensize(3)
    draw.draw_path(tesla, min_path, r)
    
    turtle.done()