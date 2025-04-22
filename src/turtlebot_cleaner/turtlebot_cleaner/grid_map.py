#!/usr/bin/env python3

import numpy as np
from enum import Enum
# import matplotlib.pyplot as plt
import os
from datetime import datetime


class CellStatus(Enum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    CLEANED = 3


class GridMap:
    def __init__(self, width, height, resolution, node, origin_x, origin_y):
        self.resolution = resolution
        self.width = width
        self.height = height
        self.node = node
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Calculate grid dimensions
        self.grid_width = int(width)
        self.grid_height = int(height)
        
        # Initialize grid with unknown cells
        self.grid = np.full((self.grid_height, self.grid_width), CellStatus.UNKNOWN.value, dtype=np.int8)

        # Initialize cleaning status grid (0: uncleaned, 1: cleaned)
        self.cleaned_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        

    def mark_cleaned(self, x, y):
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            self.cleaned_grid[grid_y, grid_x] = 1
            self.grid[grid_y, grid_x] = CellStatus.CLEANED.value
        else:
            self.node.get_logger().warn(f'Invalid grid position: ({grid_x}, {grid_y})')


    # def world_to_grid(self, x, y):
    #     grid_x = int((x / self.resolution) + (self.grid_width / 2))
    #     grid_y = int((y / self.resolution) + (self.grid_height / 2))
    #     return grid_x, grid_y
    
    def world_to_grid(self, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y


    # def grid_to_world(self, grid_x, grid_y):
    #     x = ((grid_x - (self.grid_width / 2)) * self.resolution)
    #     y = ((grid_y - (self.grid_height / 2)) * self.resolution)
    #     return x, y

    def grid_to_world(self, grid_x, grid_y):
        x = (grid_x * self.resolution) + self.origin_x
        y = (grid_y * self.resolution) + self.origin_y
        return x, y


    def get_cleaning_coverage(self):
        cleanable_cells = np.sum(self.grid == CellStatus.FREE.value) + np.sum(self.grid == CellStatus.CLEANED.value)
        if cleanable_cells == 0:
            return 0.0
        cleaned_cells = np.sum(self.grid == CellStatus.CLEANED.value)
        coverage = (cleaned_cells / cleanable_cells) * 100.0
        self.node.get_logger().warn(f'Cleaned cells: {cleaned_cells}, Cleanable cells: {cleanable_cells}, Coverage: {coverage:.3f}%')
        return coverage


    def update_from_occupancy_grid(self, occupancy_grid, threshold=50):
        # # Plot before update
        # plot_dir = "grid_plots"
        # if not os.path.exists(plot_dir):
        #     os.makedirs(plot_dir)
        # 
        # plt.figure(figsize=(10, 10))
        # plt.imshow(self.grid, cmap='viridis')
        # plt.colorbar(label='Cell Status')
        # plt.title('Grid Before Update')
        # plt.savefig(f'{plot_dir}/grid_before_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
        # plt.close()

        # Extract information from occupancy grid
        grid_width = int(occupancy_grid.info.width)
        grid_height = int(occupancy_grid.info.height)
        resolution = occupancy_grid.info.resolution

        # Check if grid dimensions match
        if grid_width != self.grid_width or grid_height != self.grid_height:
            # Reinitialize grid with new dimensions
            temp = np.full((grid_height, grid_width), CellStatus.UNKNOWN.value, dtype=np.int8)
            min_height = min(self.grid_height, grid_height)
            min_width = min(self.grid_width, grid_width)
            temp[0:min_height, 0:min_width] = self.grid[0:min_height, 0:min_width]
            self.grid = temp

            temp2 = np.zeros((grid_height, grid_width), dtype=np.int8)
            min_height = min(self.grid_height, grid_height)
            min_width = min(self.grid_width, grid_width)
            temp2[0:min_height, 0:min_width] = self.cleaned_grid[0:min_height, 0:min_width]
            self.cleaned_grid = temp2

            self.node.get_logger().warn(f'Grid dimensions updated: {self.grid_width}x{self.grid_height}x{self.resolution} -> {grid_width}x{grid_height}x{resolution}')

        self.resolution = resolution
        self.grid_width = grid_width 
        self.grid_height = grid_height
        
        data = np.array(occupancy_grid.data).reshape((self.grid_height, self.grid_width))

        # Create masks
        unknown_mask = data == -1
        obstacle_mask = data >= threshold
        free_mask = (data != -1) & (data < threshold)

        # Apply UNKNOWN and OBSTACLE values
        self.grid[unknown_mask] = CellStatus.UNKNOWN.value
        self.grid[obstacle_mask] = CellStatus.OBSTACLE.value

        # Handle FREE and CLEANED logic
        # Make a copy of the current grid to check which cells are already CLEANED
        was_cleaned_mask = self.grid == CellStatus.CLEANED.value

        # Only update FREE where it's not CLEANED
        free_not_cleaned_mask = free_mask & ~was_cleaned_mask
        self.grid[free_not_cleaned_mask] = CellStatus.FREE.value

        # Update CLEANED and cleaned_grid
        free_and_cleaned_mask = free_mask & was_cleaned_mask
        self.grid[free_and_cleaned_mask] = CellStatus.CLEANED.value
        self.cleaned_grid[free_and_cleaned_mask] = 1

        self.node.get_logger().info(f'number of total cells: {self.grid_width * self.grid_height}')

        # # Plot after update
        # plt.figure(figsize=(10, 10))
        # plt.imshow(self.grid, cmap='viridis')
        # plt.colorbar(label='Cell Status')
        # plt.title('Grid After Update')
        # plt.savefig(f'{plot_dir}/grid_after_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png')
        # plt.close()


    def update_cell(self, x, y, status, node):
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            self.grid[grid_y, grid_x] = status.value
        else:
            node.get_logger().warn(f'Invalid grid position: ({grid_x}, {grid_y})')

