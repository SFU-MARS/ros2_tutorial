#!/usr/bin/env python3

import numpy as np
import math
import random
from .grid_map import GridMap, CellStatus


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
        return [(0, -1), (1, 0), (0, 1), (-1, 0)] # 4 directions (N, E, S, W)
    
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
            if grid[grid_y, grid_x] == CellStatus.FREE.value:
                grid[grid_y, grid_x] = CellStatus.CLEANED.value
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
    def __init__(self, exploration_weight=1.0, simulation_steps=100, num_iterations=1000, robot_radius=0.2, node=None):
        self.exploration_weight = exploration_weight
        self.simulation_steps = simulation_steps
        self.num_iterations = num_iterations
        self.robot_radius = robot_radius
        self.node = node


    def print_tree(self, node, max_depth=5, text="", level=0):
        if node is None or max_depth < 0:
            return
        
        text += f"{'  ' * level} Action: {node.action}, Visits: {node.visits}, Value: {node.value:.2f}\n"  
        
        for child in node.children:
            text = self.print_tree(child, max_depth=max_depth - 1, text=text, level=level + 1)
        
        return text

    def print_tree(self, node, max_depth=5, level=0):
        def _print(node, level, remaining_depth, lines):
            if node is None or remaining_depth < 0:
                return
            
            lines.append(f"{'  ' * level} level: {level}, Action: {node.action}, Visits: {node.visits}, Value: {node.value:.2f}")
            
            for child in node.children:
                _print(child, level + 1, remaining_depth - 1, lines)

        lines = []
        _print(node, level, max_depth, lines)
        return "\n".join(lines)

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

        # self.node.get_logger().error(self.print_tree(root))
        
        # Return the action with highest visit count
        return max(root.children, key=lambda c: c.visits).action
    
    def _simulate(self, state):
        """
        Run a random simulation from the given state and return the reward
        """
        sim_state = {
            'grid': state['grid'].copy(),
            'robot_pos': state['robot_pos'],
            'uncleaned_count': state['uncleaned_count']
        }
        total_reward = 0
        step = 0
        
        # North, East, South, West
        actions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        
        while step < self.simulation_steps:
            # Check if terminal state
            if sim_state['uncleaned_count'] <= (np.sum(sim_state['grid'] == CellStatus.CLEANED.value) * (2 / 8)):
                total_reward += 100
                break
            
            # Choose a random action
            dx, dy = random.choice(actions)
            
            # Get current position
            x, y = sim_state['robot_pos']
            new_x, new_y = x + dx, y + dy
            
            grid = sim_state['grid']
            height, width = grid.shape[0], grid.shape[1]
            
            # Check if position is within grid bounds
            if 0 <= new_x < width and 0 <= new_y < height:
                # Check for collision
                collision = self.check_collision_with_obstacles(new_x, new_y, grid)
                
                if not collision and grid[new_y, new_x] == CellStatus.FREE.value:
                    sim_state['robot_pos'] = (new_x, new_y)
                    step_reward = 0
                    
                    # Reward for cleaning a new cell
                    if grid[new_y, new_x] == CellStatus.FREE.value:  # If not cleaned before
                        grid[new_y, new_x] = CellStatus.CLEANED.value  # Mark as cleaned
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
        total_cells = sim_state['uncleaned_count'] + np.sum(grid == CellStatus.CLEANED.value)
        if total_cells == 0:
            coverage_reward = 0
        else:
            cleaned_cells = np.sum(grid == CellStatus.CLEANED.value)
            coverage_percent = (cleaned_cells / total_cells) * 100 if total_cells > 0 else 0
            coverage_reward = coverage_percent * 0.5

        total_reward += coverage_reward
        
        return total_reward
    
    def check_collision_with_obstacles(self, x, y, grid, radius_cells=5):
        height, width = grid.shape
        
        # Define the slice bounds
        x_min = max(0, x - radius_cells)
        x_max = min(width, x + radius_cells + 1)
        y_min = max(0, y - radius_cells)
        y_max = min(height, y + radius_cells + 1)

        # Extract region of interest (ROI)
        region = grid[y_min:y_max, x_min:x_max]

        # Return True if any obstacle cell is found
        return np.any(region == CellStatus.OBSTACLE.value)

    
    def prepare_state(self, grid_map, robot_pos):
        """
        Prepare the state representation for MCTS from a GridMap and robot position
        """
        grid = grid_map.grid.copy()
        
        # Count uncleaned cells
        uncleaned_count = np.sum(grid == CellStatus.FREE.value)
        
        return {
            'grid': grid.copy(),
            'robot_pos': robot_pos,
            'uncleaned_count': uncleaned_count
        } 