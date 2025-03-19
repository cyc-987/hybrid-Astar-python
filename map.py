import numpy as np
import matplotlib
from matplotlib import pyplot as plt

class Map:
    def __init__(self, 
                 map_type: int=0):
        self.width = 50
        self.height = 25
        
        self.map = np.zeros((self.height, self.width), dtype=int)
        self._GenerateMap(map_type)
        
    def _GenerateMap(self, map_type: int):
        if map_type == 0:
            return
        elif map_type == 1:
            self.map[0:15, 17:19] = 1
            self.map[10:25, 34:36] = 1
        else:
            raise ValueError("Invalid map type")
    
    def MapInfo(self):
        number_of_obstacles = np.sum(self.map)
        print(f"Map size: {self.width}x{self.height}")
        print(f"Number of obstacles: {number_of_obstacles}")
        print(f"Obstacle density: {number_of_obstacles / (self.width * self.height):.2f}")
        
if __name__ == '__main__':
    print("Testing Map class")
    
    m = Map(map_type=1)
    m.MapInfo()
    
    plt.imshow(m.map, cmap='gray_r', origin='lower', vmin=0, vmax=1)
    plt.show()
    