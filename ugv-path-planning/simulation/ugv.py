"""
UGV (Unmanned Ground Vehicle) Simulation Module.

This module provides the UGV class that simulates an unmanned ground vehicle
navigating through a grid environment using pathfinding algorithms.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import time
from typing import List, Tuple, Optional, Dict, Set
from environment.grid import Grid
from environment.obstacle_generator import ObstacleGenerator
from algorithms.dijkstra import DijkstraGridNavigator
from algorithms.astar import AStarNavigator


class UGV:
    """
    Unmanned Ground Vehicle simulation for path planning.
    
    The UGV navigates from a start position to a goal position
    using pathfinding algorithms like Dijkstra or A*.
    
    Attributes:
        position: Current position of the UGV
        goal: Goal position
        grid: Grid environment
        algorithm: Pathfinding algorithm name
    """
    
    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int],
                 grid: Grid, algorithm: str = 'dijkstra'):
        """
        Initialize the UGV.
        
        Args:
            start: Starting position as (row, col)
            goal: Goal position as (row, col)
            grid: Grid environment
            algorithm: Pathfinding algorithm ('dijkstra' or 'astar')
        """
        self.start = start
        self.goal = goal
        self.position = start
        self.grid = grid
        self.algorithm = algorithm
        
        # Path and navigation state
        self.current_path: List[Tuple[int, int]] = []
        self.explored_nodes: Set[Tuple[int, int]] = set()
        self.total_cost: float = 0.0
        self.steps_taken: int = 0
        self.is_at_goal: bool = False
        
        # Initialize navigator
        self._init_navigator()
    
    def _init_navigator(self) -> None:
        """Initialize the pathfinding navigator based on algorithm choice."""
        if self.algorithm.lower() == 'dijkstra':
            self.navigator = DijkstraGridNavigator(
                self.grid.width, 
                self.grid.height
            )
        elif self.algorithm.lower() == 'astar':
            self.navigator = AStarNavigator(
                self.grid.width,
                self.grid.height
            )
        else:
            raise ValueError(f"Unknown algorithm: {self.algorithm}. "
                           f"Choose 'dijkstra' or 'astar'")
    
    def find_path(self, track_explored: bool = True) -> Optional[List[Tuple[int, int]]]:
        """
        Find a path from current position to goal.
        
        Args:
            track_explored: Whether to track explored nodes
            
        Returns:
            List of positions forming the path, or None if no path exists
        """
        path, cost, explored = self.navigator.find_path(
            self.grid.cells,
            self.position,
            self.goal,
            track_explored=track_explored
        )
        
        if path:
            self.current_path = path
            self.total_cost = cost
            self.explored_nodes = explored
            # Remove current position from path (already there)
            if path[0] == self.position:
                self.current_path = path[1:]
        
        return self.current_path if path else None
    
    def move_to_goal(self, step_delay: float = 0.0) -> bool:
        """
        Move the UGV to the goal along the current path.
        
        Args:
            step_delay: Delay between each step (seconds)
            
        Returns:
            True if goal reached, False otherwise
        """
        if not self.current_path:
            # Try to find path first
            if not self.find_path():
                return False
        
        while self.current_path:
            # Move to next position
            next_pos = self.current_path.pop(0)
            self.position = next_pos
            self.steps_taken += 1
            
            # Check if at goal
            if self.position == self.goal:
                self.is_at_goal = True
                return True
            
            # Delay between steps
            if step_delay > 0:
                time.sleep(step_delay)
        
        return self.is_at_goal
    
    def step(self) -> bool:
        """
        Take a single step towards the goal.
        
        Returns:
            True if at goal, False otherwise
        """
        if self.is_at_goal:
            return True
        
        if not self.current_path:
            # Try to find path
            if not self.find_path():
                return False
        
        if self.current_path:
            # Move to next position
            next_pos = self.current_path.pop(0)
            self.position = next_pos
            self.steps_taken += 1
            
            if self.position == self.goal:
                self.is_at_goal = True
                return True
        
        return self.is_at_goal
    
    def replan(self) -> bool:
        """
        Replan path from current position.
        
        Returns:
            True if new path found, False otherwise
        """
        # Reset path state
        self.current_path = []
        self.explored_nodes = set()
        
        # Find new path
        return self.find_path() is not None
    
    def get_status(self) -> Dict:
        """
        Get current UGV status.
        
        Returns:
            Dictionary with status information
        """
        return {
            'position': self.position,
            'goal': self.goal,
            'at_goal': self.is_at_goal,
            'path_remaining': len(self.current_path),
            'steps_taken': self.steps_taken,
            'total_cost': self.total_cost,
            'explored_count': len(self.explored_nodes),
            'algorithm': self.algorithm
        }
    
    def reset(self, start: Optional[Tuple[int, int]] = None,
              goal: Optional[Tuple[int, int]] = None) -> None:
        """
        Reset UGV to starting state.
        
        Args:
            start: New start position (uses current if None)
            goal: New goal position (uses current if None)
        """
        if start is not None:
            self.start = start
            self.position = start
        else:
            self.position = self.start
        
        if goal is not None:
            self.goal = goal
        
        self.current_path = []
        self.explored_nodes = set()
        self.total_cost = 0.0
        self.steps_taken = 0
        self.is_at_goal = False
    
    def __repr__(self) -> str:
        """String representation of UGV state."""
        return (f"UGV(position={self.position}, goal={self.goal}, "
                f"steps={self.steps_taken}, algorithm={self.algorithm})")


class UGVSsimulator:
    """
    Simulator for running UGV navigation scenarios.
    
    This class provides a higher-level interface for running
    UGV navigation simulations with various configurations.
    """
    
    def __init__(self, grid_size: Tuple[int, int] = (70, 70),
                 obstacle_density: str = 'medium',
                 algorithm: str = 'dijkstra',
                 random_seed: Optional[int] = None):
        """
        Initialize the simulator.
        
        Args:
            grid_size: (width, height) of the grid
            obstacle_density: Density level ('low', 'medium', 'high')
            algorithm: Pathfinding algorithm
            random_seed: Random seed for reproducibility
        """
        self.grid_size = grid_size
        self.obstacle_density = obstacle_density
        self.algorithm = algorithm
        self.random_seed = random_seed
        
        # Create grid and UGV
        self.grid = Grid(grid_size[0], grid_size[1])
        generator = ObstacleGenerator(self.grid, random_seed)
        generator.generate_with_density_level(obstacle_density)
        
        # Define start and goal positions
        start = (0, 0)
        goal = (grid_size[1] - 1, grid_size[0] - 1)
        
        self.ugv = UGV(start, goal, self.grid, algorithm)
    
    def run(self, step_delay: float = 0.0, 
            verbose: bool = True) -> bool:
        """
        Run the simulation.
        
        Args:
            step_delay: Delay between steps
            verbose: Whether to print status
            
        Returns:
            True if goal reached, False otherwise
        """
        # Find initial path
        path = self.ugv.find_path()
        
        if not path:
            if verbose:
                print("No path found from start to goal!")
            return False
        
        if verbose:
            print(f"Initial path found: {len(path)} steps")
            print(f"Starting navigation...")
        
        # Move to goal
        success = self.ugv.move_to_goal(step_delay)
        
        if verbose:
            status = self.ugv.get_status()
            print(f"\nNavigation {'completed' if success else 'failed'}!")
            print(f"Steps taken: {status['steps_taken']}")
            print(f"Total cost: {status['total_cost']}")
            print(f"Explored nodes: {status['explored_count']}")
        
        return success
    
    def get_grid_with_visualization_data(self) -> Dict:
        """
        Get grid with all visualization data.
        
        Returns:
            Dictionary with grid and visualization layers
        """
        return {
            'grid': self.grid,
            'obstacles': self.grid.get_obstacles(),
            'path': self.ugv.current_path,
            'explored': self.ugv.explored_nodes,
            'position': self.ugv.position,
            'goal': self.ugv.goal
        }
