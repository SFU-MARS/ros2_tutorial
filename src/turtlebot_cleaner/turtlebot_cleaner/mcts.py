#!/usr/bin/env python3

import numpy as np
import math
import random
from collections import defaultdict

class MCTSNode:
    def __init__(self, state, parent=None, action=None):
        self.state = state  # Current state (robot position, grid state)
        self.parent = parent
        self.action = action  # Action that led to this state
        self.children = []
        self.visits = 0  # Number of visits
        self.value = 0.0  # Accumulated reward
        self.untried_actions = self._get_untried_actions()  # Actions not yet tried
        
    def _get_untried_actions(self):
        """Get all possible actions from the current state that haven't been tried."""
        # 4 directions (N, E, S, W)
        return [(0, -1), (1, 0), (0, 1), (-1, 0)]
    
    def select_child(self, exploration_weight=1.0):
        """Select a child node using UCB1 formula."""
        log_visits = math.log(self.visits) if self.visits > 0 else 0
        
        def ucb(child):
            if child.visits == 0:
                return float('inf')
            return (child.value / child.visits) + exploration_weight * math.sqrt(2 * log_visits / child.visits)
        
        return max(self.children, key=ucb)
    
    def expand(self):
        """Expand the node by adding a child node for an untried action"""
        if not self.untried_actions:
            return None
        
        # Choose random untried action
        action = self.untried_actions.pop(random.randrange(len(self.untried_actions)))
        
        # Create new state by applying the action
        next_state = self._apply_action(action)
        
        # Create child node
        child = MCTSNode(next_state, parent=self, action=action)
        self.children.append(child)
        return child
    
    def _apply_action(self, action):
        """Apply action to the current state and return the next state"""
        next_state = self.state.copy()
        
        # Update robot position
        dx, dy = action
        x, y = next_state['robot_pos']
        new_x, new_y = x + dx, y + dy
        next_state['robot_pos'] = (new_x, new_y)
        
        # Mark the new position as cleaned
        grid_x, grid_y = int(new_x), int(new_y)
        grid = next_state['grid']
        if 0 <= grid_x < grid.shape[1] and 0 <= grid_y < grid.shape[0]:
            # If the cell was not cleaned before
            if grid[grid_y, grid_x] == 0:
                grid[grid_y, grid_x] = 1
                next_state['uncleaned_count'] -= 1
        
        return next_state
    
    def update(self, reward):
        self.visits += 1
        self.value += reward
    
    def is_terminal(self):
        # If all cells are cleaned or no valid moves
        return self.state['uncleaned_count'] == 0 or not self.untried_actions and not self.children
    
    def is_fully_expanded(self):
        return len(self.untried_actions) == 0

class MCTS:
    def __init__(self, exploration_weight=1.0, simulation_steps=100, num_iterations=1000, robot_radius=0.2):
        self.exploration_weight = exploration_weight
        self.simulation_steps = simulation_steps
        self.num_iterations = num_iterations
        self.robot_radius = robot_radius
    
    def search(self, initial_state):
        """
        Run the MCTS algorithm from the initial state and return the best action
        """
        root = MCTSNode(initial_state)
        
        for _ in range(self.num_iterations):
            # Selection
            node = root
            while not node.is_terminal() and node.is_fully_expanded():
                node = node.select_child(self.exploration_weight)
            
            # Expansion
            if not node.is_terminal():
                node = node.expand()
                if node is None:  # If no expansion was possible
                    continue
            
            # Simulation
            reward = self._simulate(node.state)
            
            # Backpropagation
            while node is not None:
                node.update(reward)
                node = node.parent
        
        # Select the best action from the root
        if not root.children:
            return None
        
        # Return the action with highest visit count
        return max(root.children, key=lambda c: c.visits).action
    
    def _simulate(self, state):
        """
        Run a random simulation from the given state and return the reward
        """
        sim_state = {
            'grid': state['grid'].copy(),
            'obstacle_grid': state['obstacle_grid'].copy(),
            'robot_pos': state['robot_pos'],
            'uncleaned_count': state['uncleaned_count']
        }
        total_reward = 0
        step = 0
        
        # North, East, South, West
        actions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        
        while step < self.simulation_steps:
            # Check if terminal state
            if sim_state['uncleaned_count'] == 0:
                total_reward += 100
                break
            
            # Choose a random action
            dx, dy = random.choice(actions)
            
            # Get current position
            x, y = sim_state['robot_pos']
            x, y = int(x), int(y)
            new_x, new_y = x + dx, y + dy
            
            grid = sim_state['grid']
            obstacle_grid = sim_state['obstacle_grid']
            height, width = grid.shape[0], grid.shape[1]
            
            # Check if position is within grid bounds
            if 0 <= new_x < width and 0 <= new_y < height:
                # Check for collision
                collision = self.check_collision_with_obstacles(new_x, new_y, sim_state['obstacle_grid'])
                
                if not collision and obstacle_grid[new_y, new_x] == 0:
                    sim_state['robot_pos'] = (new_x, new_y)
                    step_reward = 0
                    
                    # Reward for cleaning a new cell
                    if grid[new_y, new_x] == 0:  # If not cleaned before
                        grid[new_y, new_x] = 1  # Mark as cleaned
                        sim_state['uncleaned_count'] -= 1
                        step_reward += 10  # High reward for cleaning
                    else:
                        step_reward -= 1  # Small penalty for revisiting cleaned cells
                    
                    total_reward += step_reward
                else:
                    # Penalty for trying to move into an obstacle
                    total_reward -= 50
            else:
                # Penalty for trying to move out of bounds
                total_reward -= 25
            
            step += 1
        
        # Add final reward based on cleaning coverage
        total_cells = grid.shape[0] * grid.shape[1] - np.sum(sim_state['obstacle_grid'])
        cleaned_cells = np.sum(grid)
        coverage_percent = (cleaned_cells / total_cells) * 100 if total_cells > 0 else 0
        
        coverage_reward = coverage_percent * 0.5
        total_reward += coverage_reward
        
        return total_reward
    
    def check_collision_with_obstacles(self, x, y, obstacle_grid, radius_cells=None):
        """Check if the robot collides with any obstacles considering its radius"""
        height, width = obstacle_grid.shape
        
        if radius_cells is None:
            resolution = 0.05
            radius_cells = max(1, int(self.robot_radius / resolution))
        
        for i in range(-radius_cells, radius_cells + 1):
            for j in range(-radius_cells, radius_cells + 1):
                check_x, check_y = x + i, y + j
                # Check if cell is within grid bounds
                if 0 <= check_x < width and 0 <= check_y < height:
                    # Check if cell is an obstacle
                    if obstacle_grid[check_y, check_x] == 1:
                        return True
        
        return False
    
    def prepare_state(self, grid_map, robot_pos):
        """
        Prepare the state representation for MCTS from a GridMap and robot position
        """
        # Ensure robot_pos contains integers
        robot_pos = (int(robot_pos[0]), int(robot_pos[1]))
        
        # Create binary grids for obstacles and cleaning status
        obstacle_grid = np.zeros((grid_map.grid_height, grid_map.grid_width), dtype=np.int8)
        cleaned_grid = np.zeros((grid_map.grid_height, grid_map.grid_width), dtype=np.int8)
        
        for y in range(grid_map.grid_height):
            for x in range(grid_map.grid_width):
                if grid_map.grid[y, x] == 2:  # OBSTACLE
                    obstacle_grid[y, x] = 1
                if grid_map.cleaned_grid[y, x] == 1:  # CLEANED
                    cleaned_grid[y, x] = 1
        
        # Count uncleaned cells
        uncleaned_count = np.sum((obstacle_grid == 0) & (cleaned_grid == 0))
        
        return {
            'grid': cleaned_grid.copy(),
            'obstacle_grid': obstacle_grid,
            'robot_pos': robot_pos,
            'uncleaned_count': uncleaned_count
        } 