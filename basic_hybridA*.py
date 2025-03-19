import numpy as np
from matplotlib import pyplot as plt
import math
import heapq
import turtle

from map import Map
import reeds_shepp as rs
import draw

pi = math.pi
sin = np.sin
cos = np.cos

class Node:
    def __init__(self, x, y, theta, parent=None):
        # 位置信息
        self.x = x
        self.y = y
        self.theta = theta % (2 * pi)  # 角度限制在[0,2π)
        
        # 归属
        self.parent = parent
        self.position = (math.floor(x), math.floor(y))
        
        # 代价
        self.g_cost = 0.0
        self.h_cost = 0.0

    def f_cost(self):
        return self.g_cost + self.h_cost
    
    def __eq__(self, other):
        return self.position == other.position and self.theta == other.theta
    
    def __lt__(self, other):
        return self.f_cost() < other.f_cost() if self.f_cost() != other.f_cost() else self.h_cost < other.h_cost
    
    def __repr__(self):
        return f"({self.x:.2f}, {self.y:.2f}) θ={self.theta:.2f}"
    
class Object:
    def __init__(self):
        self.length = 4.5
        self.width = 2.0
        self.radius = 1.0

class HybridAStar:
    def __init__(self,
                 start: tuple,
                 goal: tuple,
                 map: Map,
                 object: Object):
        # 起点和终点
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map = map
        self.car = object
        
        # 开放列表和关闭列表
        self.open_list = [] # 优先队列
        self.open_dict = {} # 字典
        self.closed_dict = {}
        
        # 路径
        self.path = []
        
        # 计算起点的启发代价
        self.start.h_cost = self.CalcHeuristic(self.start)
        heapq.heappush(self.open_list, self.start)
        self.open_dict[self.start.position] = self.start
    
    def CalcHeuristic(self, node: Node):
        '''计算节点到目标节点的启发代价
        Args:
            node: 当前节点
        Returns:
            最短RS曲线和欧几里得距离的最大值
        '''
        start = (node.x, node.y, node.theta)
        end = (self.goal.x, self.goal.y, self.goal.theta)
        
        min_rs_path, _ = rs.RSCurve(*start, self.car.radius)(*end)
        min_rs_len = rs.PathLen(min_rs_path, self.car.radius)
        
        eu_dist = math.hypot(node.x - self.goal.x, node.y - self.goal.y)
        
        # print(f"min_rs_len: {min_rs_len:.2f}, eu_dist: {eu_dist:.2f}")
        return max(min_rs_len, eu_dist)
    
    def Perform(self, max_attempts: int=10000):
        ''' 执行算法
        Args:
            max_attempts: 最大尝试次数
        Returns:
            path, rs_path
        '''
        count = 0
        
        # 主循环体
        while(self.open_list and count < max_attempts):
            # 获取当前节点（优先队列保证最小），并将其加入关闭列表
            current_node = heapq.heappop(self.open_list)
            self.open_dict.pop(current_node.position)
            self.closed_dict[current_node.position] = current_node
            count += 1
            if count >= 300 and count % 300 == 0:
                print(f"count: {count}")
            
            # 如果找到目标节点，返回路径，此时并没有RS曲线出现
            if current_node == self.goal:
                self.path = self.GetPath(current_node)
                print(f"Path found in {count} iterations")
                return self.path
            
            # 生成当前节点的邻居节点
            neighbors = self.GetNeighbors(current_node)
            
            # 遍历邻居节点
            for neighbor in neighbors:
                # 如果在close列表，跳过
                if neighbor.position in self.closed_dict:
                    continue
                
                # 检测是否存在合法的RS曲线
                rs_path = self.CheckIfNodeHasValidRS(neighbor)
                if rs_path:
                    # 返回路径
                    path1 = self.GetPath(neighbor)
                    print(f"Path found in {count} iterations")
                    return path1, rs_path
                
                # 计算g、h和f值
                neighbor.g_cost = current_node.g_cost + math.hypot(neighbor.x - current_node.x, neighbor.y - current_node.y)
                neighbor.h_cost = self.CalcHeuristic(neighbor)
                
                # 如果节点已经在开放列表中但是新的代价更小，更新代价
                if neighbor.position in self.open_dict:
                    existing_node = self.open_dict[neighbor.position]
                    if neighbor.g_cost < existing_node.g_cost:
                        self.open_list.remove(existing_node)
                        heapq.heapify(self.open_list)
                        heapq.heappush(self.open_list, neighbor)
                        self.open_dict[neighbor.position] = neighbor
                else:
                    heapq.heappush(self.open_list, neighbor)
                    self.open_dict[neighbor.position] = neighbor
                    
        print(f"Path not found in {count} iterations")
        return None, None
    
    def SmoothPath(self, path: list):
        '''平滑路径
        Args:
            path: 一系列点坐标列表
        Returns:
            RS曲线列表
        '''
        final = []
        current_index = 0;
        max_index = len(path) - 1
        print(f"max index: {max_index}")
        
        while current_index < max_index:
            # 从当前点到最后一个点的RS曲线
            start = path[current_index]
            end_index = max_index
            
            while end_index > current_index:
                end = path[end_index]
                min_rs_path, all_paths = rs.RSCurve(*start, self.car.radius)(*end)
                
                # 检查RS曲线是否合法
                if self._CheckRSCollision(start, min_rs_path):
                    all_failed = True
                    for p in all_paths:
                        if self._CheckRSCollision(start, p):
                            continue
                        else:
                            all_failed = False
                            break
                else: p = min_rs_path
                
                if all_failed:
                    end_index -= 1
                else:
                    print(f"RS path found from {current_index} to {end_index}")
                    break
            
            # 添加RS曲线
            final.append(p)
            current_index = end_index
            
        return final
                
                
                
    def CheckIfNodeHasValidRS(self, node: Node):
        '''检查节点是否存在合法的RS曲线
        Args:
            node: 当前节点
        Returns:
            是否存在合法的RS曲线，如果存在，返回该曲线
        '''
        start = (node.x, node.y, node.theta)
        end = (self.goal.x, self.goal.y, self.goal.theta)
        path_min, _ = rs.RSCurve(*start, self.car.radius)(*end)
        
        if self._CheckRSCollision(start, path_min):
            return False
        return path_min
        
    def _CheckRSCollision(self, start, path: list):
        '''检查RS曲线是否和障碍物产生碰撞
        Args:
            start: 起点(x, y, theta)
            path: RS曲线
        Returns:
            是否和障碍物碰撞
        '''
        sample_interval = 0.1
        x, y, theta = start
        
        for segment in path: # 对于path列表的每一个构成元素
            segment_length = segment.value * self.car.radius
            steering = segment.steering  # 转向角（-1左, 0直行, 1右）
            gear = segment.gear  # 档位（-1后退, 1前进）
            
            # 根据segment长度确定采样点数量
            num_samples = max(2, int(segment_length / sample_interval))
            
            for i in range(num_samples):
                # 计算当前位置在segment上的比例
                ratio = i / (num_samples - 1)
                
                # 计算新位置
                if steering == 0:  # 直线
                    new_x = x + ratio * segment_length * cos(theta) * gear
                    new_y = y + ratio * segment_length * sin(theta) * gear
                    new_theta = theta
                    node_new = Node(new_x, new_y, new_theta)
                    
                else:  # 圆弧
                    # 计算旋转中心
                    turn_radius = self.car.radius
                    center_x = x + turn_radius * sin(theta) * steering
                    center_y = y - turn_radius * cos(theta) * steering
                    
                    # 计算旋转角度
                    angle = ratio * segment.value * steering * gear
                    
                    # 旋转后的位置
                    node_new = self.Rotate(Node(x, y, theta), (center_x, center_y), -angle, turn_radius)
                
                # 判断
                if not self._CheckNodeValid(node_new):
                    return True
            
            # 更新位置
            x, y, theta = node_new.x, node_new.y, node_new.theta
            
        # 没有碰撞
        return False
    
    def GetPath(self, node: Node):
        '''返回不带RS曲线的路径
        Args:
            node: 当前节点
        Returns:
            路径
        '''
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y, current.theta))
            current = current.parent
        return path[::-1]
    
    def GetNeighbors(self, node: Node):
        '''获取当前节点的邻居节点，保证生成的节点在地图内且无障碍物
        Args:
            node: 当前节点
        Returns:
            邻居节点列表（包括前进、后退、前后左转、前后右转）
        '''
        neighbors = []
        # 步长为栅格对角线
        step = math.sqrt(2)
        # 前后方向
        node_forward_straight = Node(node.x + step * cos(node.theta), 
                                     node.y + step * sin(node.theta),
                                     node.theta, node)
        node_forward_back = Node(node.x - step * cos(node.theta),
                                 node.y - step * sin(node.theta),
                                 node.theta, node)
        # 计算旋转原点
        radius = self.car.radius
        o1 = (node.x + radius * sin(node.theta),
              node.y - radius * cos(node.theta)) # 右下角
        o2 = (node.x - radius * sin(node.theta),
              node.y + radius * cos(node.theta)) # 左上角
        
        # 计算四个离散点
        node_back_right = self.Rotate(node, o1, (step/self.car.radius), self.car.radius*2) # 右转后退
        node_forward_right = self.Rotate(node, o1, -(step/self.car.radius), self.car.radius*2) # 右转前进
        node_forward_left = self.Rotate(node, o2, (step/self.car.radius), self.car.radius*2) # 左转前进
        node_back_left = self.Rotate(node, o2, -(step/self.car.radius), self.car.radius*2) # 左转后退
        
        # 合法性
        for n in [node_forward_straight, node_forward_back, node_forward_right, node_forward_left, node_back_right, node_back_left]:
            if self._CheckNodeValid(n): neighbors.append(n)
        
        return neighbors
    
    def _CheckNodeInMap(self, node: Node):
        '''检查节点是否在地图内
        Args:
            node: 当前节点
        Returns:
            是否在地图内
        '''
        return node.x >= 0 and node.x < self.map.width and node.y >= 0 and node.y < self.map.height
        
    def _CheckNodeCollision(self, node: Node):
        '''检查节点是否与障碍物碰撞
        Args:
            node: 当前节点
        Returns:
            是否与障碍物碰撞
        '''
        return self.map.map[node.position[1], node.position[0]] == 1
    
    def _CheckNodeValid(self, node: Node):
        '''检查节点是否有效
        Args:
            node: 当前节点
        Returns:
            是否有效
        '''
        if not self._CheckNodeInMap(node):
            return False
        if self._CheckNodeCollision(node):
            return False
        return True
    
    def Rotate(self, node: Node, o: tuple, angle: float, r: float=-1):
        '''向量旋转（正方向为逆时针）
        Args:
            node: 当前节点
            o: 旋转原点
            angle: 旋转角度（rad）
            r: 旋转半径（如果不给会计算）
        Returns:
            旋转后的节点
        '''
        if r == -1:
            r = math.hypot(node.x - o[0], node.y - o[1])
        
        x_normal = node.x - o[0]
        y_normal = node.y - o[1]
        
        R = np.array([[cos(angle), -sin(angle)],
                      [sin(angle), cos(angle)]])
        [x_new, y_new] = np.dot(R, [x_normal, y_normal])
        
        theta_new = node.theta + angle
        return Node(x_new + o[0], y_new + o[1], theta_new, node)
            
    
    def __call__(self):
        pass
        

    
def main():
    # 创建地图
    m = Map(map_type=1)
    m.MapInfo()
    tesla, offsetx, offsety = draw.InitAndDrawMap(m)
    car = Object()
    
    # 设置起点和终点
    offset = (offsetx, offsety, 0)
    start = (1, 1, pi/2)
    goal = (m.width-2, m.height-2, pi/2)
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