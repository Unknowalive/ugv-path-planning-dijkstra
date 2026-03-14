"""
Dynamic Replanning Simulation for UGV Navigation.

This module implements dynamic obstacle handling and path replanning
for UGV navigation. When obstacles appear during navigation, the
system detects the blockage and recomputes a new path.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import random
import time
from typing import List, Tuple, Optional, Dict, Set
from environment.grid import Grid
from environment.obstacle_generator import ObstacleGenerator
from algorithms.dijkstra import DijkstraGridNavigator
from algorithms.astar import AStarNavigator


class DynamicObstacleManager:
    """
    Manages dynamic obstacles during UGV navigation.
    
    This class handles the creation and management of obstacles
    that appear during navigation.
    """
    
    def __init__(self, grid: Grid, obstacle_probability: float = 0.05,
                 max_obstacles: int = 3, random_seed: Optional[int] = None):
        """
        Initialize the dynamic obstacle manager.
        
        Args:
            grid: Grid environment
            obstacle_probability: Probability of obstacle appearing per step
            max_obstacles: Maximum number of obstacles that can appear
            random_seed: Random seed for reproducibility
        """
        self.grid = grid
        self.obstacle_probability = obstacle_probability
        self.max_obstacles = max_obstacles
        self.generator = ObstacleGenerator(grid, random_seed)
        
        # Track dynamic obstacles
        self.dynamic_obstacles: Set[Tuple[int, int]] = set()
        self.total_dynamic_obstacles_added: int = 0
        self.obstacles_blocking_path: int = 0
    
    def attempt_add_obstacle(self, avoid_positions: Set[Tuple[int, int]],
                            current_path: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        """
        Attempt to add a dynamic obstacle.
        
        Args:
            avoid_positions: Positions to avoid
            current_path: Current planned path
            
        Returns:
            Position of new obstacle, or None if not added
        """
        # Check if we should add an obstacle
        if random.random() > self.obstacle_probability:
            return None
        
        if len(self.dynamic_obstacles) >= self.max_obstacles:
            return None
        
        # Add obstacle avoiding current path
        new_obstacle = self.generator.add_dynamic_obstacle(avoid_positions)
        
        if new_obstacle:
            self.dynamic_obstacles.add(new_obstacle)
            self.total_dynamic_obstacles_added += 1
            
            # Check if it blocks the path
            if new_obstacle in current_path:
                self.obstacles_blocking_path += 1
        
        return new_obstacle
    
    def reset(self) -> None:
        """Reset the dynamic obstacle manager."""
        # Remove all dynamic obstacles from grid
        for pos in self.dynamic_obstacles:
            self.grid.set_free(pos)
        
        self.dynamic_obstacles.clear()
        self.total_dynamic_obstacles_added = 0
        self.obstacles_blocking_path = 0
    
    def get_statistics(self) -> Dict:
        """Get statistics about dynamic obstacles."""
        return {
            'total_added': self.total_dynamic_obstacles_added,
            'currently_active': len(self.dynamic_obstacles),
            'blocking_path': self.obstacles_blocking_path
        }


class DynamicReplanningNavigator:
    """
    UGV navigator with dynamic replanning capabilities.
    
    This class extends the basic navigation with the ability to
    detect path blockages and replan when obstacles appear.
    """
    
    def __init__(self, grid: Grid, start: Tuple[int, int], goal: Tuple[int, int],
                 algorithm: str = 'dijkstra',
                 obstacle_probability: float = 0.05,
                 max_dynamic_obstacles: int = 3,
                 random_seed: Optional[int] = None):
        """
        Initialize the dynamic replanning navigator.
        
        Args:
            grid: Grid environment
            start: Starting position
            goal: Goal position
            algorithm: Pathfinding algorithm
            obstacle_probability: Probability of dynamic obstacles
            max_dynamic_obstacles: Maximum dynamic obstacles
            random_seed: Random seed
        """
        self.grid = grid
        self.start = start
        self.goal = goal
        self.algorithm = algorithm
        
        # Initialize navigator
        if algorithm.lower() == 'dijkstra':
            self.navigator = DijkstraGridNavigator(grid.width, grid.height)
        else:
            self.navigator = AStarNavigator(grid.width, grid.height)
        
        # Initialize dynamic obstacle manager
        self.obstacle_manager = DynamicObstacleManager(
            grid, obstacle_probability, max_dynamic_obstacles, random_seed
        )
        
        # Navigation state
        self.current_path: List[Tuple[int, int]] = []
        self.explored_nodes: Set[Tuple[int, int]] = set()
        self.position = start
        self.is_at_goal = bool = False
        self.steps_taken: int = 0
        self.replan_count: int = 0
        
        # Find initial path
        self._find_initial_path()
    
    def _find_initial_path(self) -> bool:
        """Find the initial path from start to goal."""
        path, cost, explored = self.navigator.find_path(
            self.grid.cells,
            self.position,
            self.goal,
            track_explored=True
        )
        
        if path:
            self.current_path = path
            self.explored_nodes = explored
            # Remove current position from path
            if path[0] == self.position:
                self.current_path = path[1:]
            return True
        
        return False
    
    def _is_path_blocked(self) -> bool:
        """
        Check if the current path is blocked by obstacles.
        
        Returns:
            True if path is blocked, False otherwise
        """
        for pos in self.current_path:
            if self.grid.is_obstacle(pos):
                return True
        return False
    
    def _find_new_path(self) -> bool:
        """
        Find a new path from current position to goal.
        
        Returns:
            True if new path found, False otherwise
        """
        # Find new path
        path, cost, explored = self.navigator.find_path(
            self.grid.cells,
            self.position,
            self.goal,
            track_explored=True
        )
        
        if path:
            self.current_path = path
            self.explored_nodes = explored
            self.replan_count += 1
            
            # Remove current position from path
            if path[0] == self.position:
                self.current_path = path[1:]
            
            return True
        
        return False
    
    def step(self, add_dynamic_obstacles: bool = True) -> Dict:
        """
        Take a single step in the simulation.
        
        Args:
            add_dynamic_obstacles: Whether to attempt adding dynamic obstacles
            
        Returns:
            Dictionary with step information
        """
        result = {
            'position': self.position,
            'at_goal': self.is_at_goal,
            'obstacle_added': None,
            'replanned': False,
            'path_blocked': False
        }
        
        if self.is_at_goal:
            return result
        
        # Check if path is blocked
        if self._is_path_blocked():
            result['path_blocked'] = True
            
            # Try to replan
            if self._find_new_path():
                result['replanned'] = True
            else:
                # No path found - stuck
                return result
        
        # Attempt to add dynamic obstacles
        if add_dynamic_obstacles:
            avoid_positions = set(self.current_path) if self.current_path else set()
            avoid_positions.add(self.position)
            avoid_positions.add(self.goal)
            
            new_obstacle = self.obstacle_manager.attempt_add_obstacle(
                avoid_positions, self.current_path
            )
            
            if new_obstacle:
                result['obstacle_added'] = new_obstacle
                
                # Check if new obstacle blocks path
                if new_obstacle in self.current_path:
                    result['path_blocked'] = True
                    
                    # Replan if blocked
                    if self._find_new_path():
                        result['replanned'] = True
        
        # Move along path if available
        if self.current_path:
            self.position = self.current_path.pop(0)
            self.steps_taken += 1
            
            # Check if at goal
            if self.position == self.goal:
                self.is_at_goal = True
        
        result['position'] = self.position
        result['at_goal'] = self.is_at_goal
        
        return result
    
    def run_simulation(self, max_steps: int = 5000,
                       step_delay: float = 0.0,
                       verbose: bool = True) -> bool:
        """
        Run the complete simulation.
        
        Args:
            max_steps: Maximum number of steps
            step_delay: Delay between steps
            verbose: Whether to print progress
            
        Returns:
            True if goal reached, False otherwise
        """
        if verbose:
            print("Starting dynamic replanning simulation...")
            print(f"Start: {self.start}, Goal: {self.goal}")
            print(f"Algorithm: {self.algorithm}")
            print(f"Obstacle probability: {self.obstacle_manager.obstacle_probability}")
            print()
        
        step = 0
        while step < max_steps and not self.is_at_goal:
            result = self.step()
            
            if verbose and step % 100 == 0:
                print(f"Step {step}: Position {result['position']}, "
                      f"At goal: {result['at_goal']}")
                
                if result['obstacle_added']:
                    print(f"  -> Dynamic obstacle added at {result['obstacle_added']}")
                if result['replanned']:
                    print(f"  -> Path replanned! (Total replans: {self.replan_count})")
            
            step += 1
            
            if step_delay > 0:
                time.sleep(step_delay)
        
        # Print final statistics
        if verbose:
            print("\n" + "="*50)
            print("Simulation Complete!")
            print(f"Goal reached: {self.is_at_goal}")
            print(f"Total steps: {self.steps_taken}")
            print(f"Replan count: {self.replan_count}")
            
            stats = self.obstacle_manager.get_statistics()
            print(f"Dynamic obstacles added: {stats['total_added']}")
            print(f"Obstacles blocking path: {stats['blocking_path']}")
        
        return self.is_at_goal
    
    def get_current_state(self) -> Dict:
        """Get current simulation state."""
        return {
            'position': self.position,
            'goal': self.goal,
            'at_goal': self.is_at_goal,
            'path_remaining': len(self.current_path),
            'steps_taken': self.steps_taken,
            'replan_count': self.replan_count,
            'dynamic_obstacles': self.obstacle_manager.dynamic_obstacles.copy(),
            'current_path': self.current_path.copy()
        }
    
    def reset(self) -> None:
        """Reset the simulation."""
        self.obstacle_manager.reset()
        self.position = self.start
        self.is_at_goal = False
        self.steps_taken = 0
        self.replan_count = 0
        self._find_initial_path()


def demonstrate_dynamic_replanning(grid_size: Tuple[int, int] = (20, 20),
                                     start: Tuple[int, int] = (0, 0),
                                     goal: Tuple[int, int] = (19, 19),
                                     algorithm: str = 'dijkstra',
                                     obstacle_density: float = 0.15,
                                     obstacle_probability: float = 0.1,
                                     verbose: bool = True) -> bool:
    """
    Demonstrate dynamic replanning behavior.
    
    This function creates a small grid scenario to demonstrate
    how the UGV handles obstacles that appear during navigation.
    
    Args:
        grid_size: Size of the grid
        start: Start position
        goal: Goal position
        algorithm: Pathfinding algorithm
        obstacle_density: Initial obstacle density
        obstacle_probability: Probability of dynamic obstacles
        verbose: Whether to print progress
        
    Returns:
        True if goal reached
    """
    # Create grid with initial obstacles
    grid = Grid(grid_size[0], grid_size[1])
    generator = ObstacleGenerator(grid, random_seed=42)
    generator.generate_random(obstacle_density, ensure_connectivity=True)
    
    # Create navigator
    navigator = DynamicReplanningNavigator(
        grid, start, goal, algorithm,
        obstacle_probability=obstacle_probability,
        max_dynamic_obstacles=5,
        random_seed=42
    )
    
    # Run simulation
    return navigator.run_simulation(verbose=verbose)
