"""
Obstacle Generator for UGV Path Planning.

This module provides functionality to randomly generate obstacles
in the grid environment with configurable density levels.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import random
from typing import List, Tuple, Set, Optional
from environment.grid import Grid


class ObstacleGenerator:
    """
    Generator for creating random obstacles in a grid.
    
    This class provides various methods for generating obstacles
    with different patterns and density levels.
    
    Attributes:
        grid: The grid environment
        random_seed: Seed for random number generation (for reproducibility)
    """
    
    # Density levels with corresponding probabilities
    DENSITY_LEVELS = {
        'low': 0.15,
        'medium': 0.25,
        'high': 0.40
    }
    
    def __init__(self, grid: Grid, random_seed: Optional[int] = None):
        """
        Initialize the obstacle generator.
        
        Args:
            grid: Grid environment to generate obstacles in
            random_seed: Seed for random number generation
        """
        self.grid = grid
        if random_seed is not None:
            random.seed(random_seed)
    
    def generate_random(self, density: float = 0.25, 
                        ensure_connectivity: bool = True) -> Set[Tuple[int, int]]:
        """
        Generate random obstacles with specified density.
        
        Args:
            density: Probability of each cell being an obstacle (0.0 to 1.0)
            ensure_connectivity: Whether to ensure path exists from start to goal
            
        Returns:
            Set of obstacle positions
        """
        obstacles = set()
        
        # Generate random obstacles
        for row in range(self.grid.height):
            for col in range(self.grid.width):
                if random.random() < density:
                    obstacles.add((row, col))
        
        # Optionally ensure connectivity
        if ensure_connectivity:
            obstacles = self._ensure_path_exists(obstacles)
        
        # Apply to grid
        self._apply_obstacles(obstacles)
        
        return obstacles
    
    def generate_with_density_level(self, level: str = 'medium',
                                      ensure_connectivity: bool = True) -> Set[Tuple[int, int]]:
        """
        Generate obstacles using predefined density levels.
        
        Args:
            level: Density level ('low', 'medium', or 'high')
            ensure_connectivity: Whether to ensure path exists
            
        Returns:
            Set of obstacle positions
        """
        if level not in self.DENSITY_LEVELS:
            raise ValueError(f"Invalid density level: {level}. "
                           f"Choose from: {list(self.DENSITY_LEVELS.keys())}")
        
        density = self.DENSITY_LEVELS[level]
        return self.generate_random(density, ensure_connectivity)
    
    def generate_with_count(self, count: int,
                            ensure_connectivity: bool = True) -> Set[Tuple[int, int]]:
        """
        Generate exact number of obstacles.
        
        Args:
            count: Number of obstacles to generate
            ensure_connectivity: Whether to ensure path exists
            
        Returns:
            Set of obstacle positions
        """
        total_cells = self.grid.width * self.grid.height
        if count > total_cells:
            raise ValueError(f"Cannot generate {count} obstacles in "
                           f"grid of size {total_cells}")
        
        # Generate random positions
        all_positions = [(r, c) for r in range(self.grid.height) 
                        for c in range(self.grid.width)]
        obstacles = set(random.sample(all_positions, count))
        
        # Optionally ensure connectivity
        if ensure_connectivity:
            obstacles = self._ensure_path_exists(obstacles)
        
        # Apply to grid
        self._apply_obstacles(obstacles)
        
        return obstacles
    
    def generate_clustered(self, num_clusters: int = 10, 
                          cluster_size_range: Tuple[int, int] = (5, 15),
                          ensure_connectivity: bool = True) -> Set[Tuple[int, int]]:
        """
        Generate obstacles in clustered patterns.
        
        Args:
            num_clusters: Number of obstacle clusters
            cluster_size_range: (min, max) size for each cluster
            ensure_connectivity: Whether to ensure path exists
            
        Returns:
            Set of obstacle positions
        """
        obstacles = set()
        free_cells = self.grid.get_free_cells()
        
        for _ in range(num_clusters):
            if not free_cells:
                break
            
            # Start cluster at random position
            cluster_center = random.choice(free_cells)
            cluster_size = random.randint(*cluster_size_range)
            
            # Generate cluster
            cluster_cells = self._generate_cluster(cluster_center, cluster_size)
            obstacles.update(cluster_cells)
            
            # Update free cells
            free_cells = [pos for pos in free_cells if pos not in obstacles]
        
        # Optionally ensure connectivity
        if ensure_connectivity:
            obstacles = self._ensure_path_exists(obstacles)
        
        # Apply to grid
        self._apply_obstacles(obstacles)
        
        return obstacles
    
    def _generate_cluster(self, center: Tuple[int, int], 
                         size: int) -> Set[Tuple[int, int]]:
        """
        Generate a cluster of obstacles around a center point.
        
        Args:
            center: Center position of cluster
            size: Approximate size of cluster
            
        Returns:
            Set of positions in the cluster
        """
        cluster = set()
        queue = [center]
        visited = {center}
        
        while queue and len(cluster) < size:
            current = queue.pop(0)
            cluster.add(current)
            
            # Add neighbors with probability
            row, col = current
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    
                    new_pos = (row + dr, col + dc)
                    if (self.grid.is_valid_position(new_pos) and 
                        new_pos not in visited and
                        random.random() < 0.7):
                        visited.add(new_pos)
                        queue.append(new_pos)
        
        return cluster
    
    def _ensure_path_exists(self, obstacles: Set[Tuple[int, int]]) -> Set[Tuple[int, int]]:
        """
        Remove obstacles until a path exists from (0,0) to (height-1, width-1).
        
        This uses a simple BFS to check connectivity.
        
        Args:
            obstacles: Set of obstacle positions
            
        Returns:
            Modified set of obstacles with connectivity ensured
        """
        # Use positions from config or default
        start = (0, 0)
        goal = (self.grid.height - 1, self.grid.width - 1)
        
        # Check if path exists
        if self._check_path_exists(obstacles, start, goal):
            return obstacles
        
        # Remove obstacles until path exists
        for obstacle in list(obstacles):
            obstacles.discard(obstacle)
            if self._check_path_exists(obstacles, start, goal):
                break
        
        return obstacles
    
    def _check_path_exists(self, obstacles: Set[Tuple[int, int]],
                           start: Tuple[int, int], 
                           goal: Tuple[int, int]) -> bool:
        """
        Check if a path exists using BFS.
        
        Args:
            obstacles: Set of obstacle positions
            start: Start position
            goal: Goal position
            
        Returns:
            True if path exists, False otherwise
        """
        if start in obstacles or goal in obstacles:
            return False
        
        visited = {start}
        queue = [start]
        
        while queue:
            current = queue.pop(0)
            
            if current == goal:
                return True
            
            # Check neighbors
            row, col = current
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_pos = (row + dr, col + dc)
                
                if (self.grid.is_valid_position(new_pos) and
                    new_pos not in obstacles and
                    new_pos not in visited):
                    visited.add(new_pos)
                    queue.append(new_pos)
        
        return False
    
    def _apply_obstacles(self, obstacles: Set[Tuple[int, int]]) -> None:
        """
        Apply obstacle set to the grid.
        
        Args:
            obstacles: Set of obstacle positions
        """
        self.grid.reset()  # Clear existing obstacles
        
        for pos in obstacles:
            self.grid.set_obstacle(pos)
    
    def add_dynamic_obstacle(self, avoid_positions: Optional[Set[Tuple[int, int]]] = None) -> Optional[Tuple[int, int]]:
        """
        Add a single dynamic obstacle at a random free position.
        
        Args:
            avoid_positions: Positions to avoid when placing obstacle
            
        Returns:
            Position of added obstacle, or None if no position available
        """
        free_cells = self.grid.get_free_cells()
        
        if avoid_positions:
            free_cells = [pos for pos in free_cells if pos not in avoid_positions]
        
        if not free_cells:
            return None
        
        new_obstacle = random.choice(free_cells)
        self.grid.set_obstacle(new_obstacle)
        
        return new_obstacle
    
    def remove_obstacle(self, pos: Tuple[int, int]) -> bool:
        """
        Remove an obstacle at a position.
        
        Args:
            pos: Position to clear
            
        Returns:
            True if obstacle was removed, False otherwise
        """
        if self.grid.is_obstacle(pos):
            self.grid.set_free(pos)
            return True
        return False


def generate_grid_with_obstacles(width: int = 70, height: int = 70,
                                  density: str = 'medium',
                                  random_seed: Optional[int] = None,
                                  ensure_connectivity: bool = True) -> Grid:
    """
    Convenience function to generate a grid with obstacles.
    
    Args:
        width: Grid width
        height: Grid height
        density: Density level ('low', 'medium', 'high') or float
        random_seed: Random seed for reproducibility
        ensure_connectivity: Whether to ensure path exists
        
    Returns:
        Grid with obstacles
    """
    grid = Grid(width, height)
    generator = ObstacleGenerator(grid, random_seed)
    
    if isinstance(density, str):
        generator.generate_with_density_level(density, ensure_connectivity)
    else:
        generator.generate_random(density, ensure_connectivity)
    
    return grid
