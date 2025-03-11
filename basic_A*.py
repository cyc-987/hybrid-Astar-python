import numpy as np
import pygame
import sys
import math

# 定义地图大小
MAP_WIDTH = 30
MAP_HEIGHT = 30
TILE_SIZE = 20  # 每个格子的大小

# 定义颜色
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# 定义节点类
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  # 从起点到当前节点的代价
        self.h = 0  # 从当前节点到目标节点的启发式代价
        self.f = 0  # f = g + h

    def __eq__(self, other):
        return self.position == other.position

# A*算法实现
def astar(maze, start, end):
    # 创建开始节点和目标节点
    start_node = Node(None, start)
    end_node = Node(None, end)

    # 初始化开放列表和关闭列表
    open_list = []
    closed_list = []
    count = 0

    # 将开始节点加入开放列表
    open_list.append(start_node)

    # 循环直到找到目标节点或开放列表为空
    while open_list:
        # 获取当前节点（f值最小的节点）
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # 从开放列表中移除当前节点，并将其加入关闭列表
        open_list.pop(current_index)
        closed_list.append(current_node)
        count += 1
        if(count >= 300 and count % 300 == 0):
            print("count: ", count)

        # 如果找到目标节点，返回路径
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # 返回反转的路径

        # 生成当前节点的邻居节点
        neighbors = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # 8个方向
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # 确保邻居节点在地图范围内
            if node_position[0] < 0 or node_position[0] >= len(maze) or node_position[1] < 0 or node_position[1] >= len(maze[0]):
                continue

            # 确保邻居节点不是障碍物
            if maze[node_position[0]][node_position[1]] == 1:
                continue

            # 创建新的邻居节点
            new_node = Node(current_node, node_position)
            if any(closed_node.position == new_node.position for closed_node in closed_list):
                continue
            neighbors.append(new_node)

        # 遍历邻居节点
        for neighbor in neighbors:
            # 如果邻居节点在关闭列表中，跳过
            if any(closed_node.position == neighbor.position for closed_node in closed_list):
                continue

            # 计算g、h和f值
            neighbor.g = current_node.g + math.hypot(neighbor.position[0] - current_node.position[0], neighbor.position[1] - current_node.position[1])
            neighbor.h = math.hypot(neighbor.position[0] - end_node.position[0], neighbor.position[1] - end_node.position[1])
            neighbor.f = neighbor.g + neighbor.h

            # 如果邻居节点已经在开放列表中但是新节点更小的g值，覆盖
            in_open_list = False
            for i, open_node in enumerate(open_list):
                if open_node.position == neighbor.position:
                    in_open_list = True
                    if neighbor.g < open_node.g:
                        open_list[i] = neighbor
                    break
                
            # 将邻居节点加入开放列表
            if not in_open_list:
                open_list.append(neighbor)

    # 如果找不到路径，返回None
    return None

# 生成随机地图
def generate_map(width, height, obstacle_density=0.3):
    maze = np.random.choice([0, 1], size=(width, height), p=[1 - obstacle_density, obstacle_density])
    return maze

# 使用pygame进行可视化
def visualize_maze(maze, path, start, end):
    pygame.init()
    screen = pygame.display.set_mode((MAP_WIDTH * TILE_SIZE, MAP_HEIGHT * TILE_SIZE))
    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # 绘制地图
        screen.fill(WHITE)
        for x in range(MAP_WIDTH):
            for y in range(MAP_HEIGHT):
                if maze[x][y] == 1:
                    pygame.draw.rect(screen, BLACK, (y * TILE_SIZE, x * TILE_SIZE, TILE_SIZE, TILE_SIZE))

        # 绘制路径
        if path:
            for position in path:
                pygame.draw.rect(screen, BLUE, (position[1] * TILE_SIZE, position[0] * TILE_SIZE, TILE_SIZE, TILE_SIZE))

        # 绘制起点和终点
        pygame.draw.rect(screen, GREEN, (start[1] * TILE_SIZE, start[0] * TILE_SIZE, TILE_SIZE, TILE_SIZE))
        pygame.draw.rect(screen, RED, (end[1] * TILE_SIZE, end[0] * TILE_SIZE, TILE_SIZE, TILE_SIZE))

        pygame.display.flip()
        clock.tick(30)

# 主函数
def main():
    # 生成地图
    maze = generate_map(MAP_WIDTH, MAP_HEIGHT)

    # 定义起点和终点
    start = (0, 0)
    end = (MAP_WIDTH - 1, MAP_HEIGHT - 1)

    # 确保起点和终点不是障碍物
    maze[start[0]][start[1]] = 0
    maze[end[0]][end[1]] = 0

    # 使用A*算法寻找路径
    path = astar(maze, start, end)

    # 可视化地图和路径
    visualize_maze(maze, path, start, end)

if __name__ == "__main__":
    main()
