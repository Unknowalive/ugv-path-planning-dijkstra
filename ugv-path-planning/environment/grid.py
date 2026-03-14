"""
Grid Environment for UGV Path Planning.

This module provides a Grid class that represents the 2D environment
for UGV navigation. It handles grid initialization, obstacle management,
and cell state tracking.

Author: UGV Path Planning Team
Version: 1.0.0
"""

from typing import List, Tuple, Set, Optional, Dict
import random


class Grid:
    """
    2D Grid environment for UGV path planning.
    
    The grid represents the navigation environment where:
    - 0 represents a free cell (passable)
    - 1 represents an obstacle (impassable)
    
    Attributes:
        width: Width of the grid (number of columns)
        height: Height of the grid (number of rows)
        cells: 2D list representing grid cells
    """
    
    def __init__(self, width: int = 70, height: int = 70):
        """
        Initialize the grid environment.
        
        Args:
            width: Width of the grid (number of columns)
            height: Height of the grid (number of rows)
        """
        self.width = width
        self.height = height
        self.cells: List[List[int]] = self._create_empty_grid()
    
    def _create_empty_grid(self) -> List[List[int]]:
        """
        Create an empty grid with all free cells.
        
        Returns:
            2D list initialized with zeros (free cells)
        """
        return [[0 for _ in range(self.width)] for _ in range(self.height)]
    
    def reset(self) -> None:
        """Reset the grid to all free cells."""
        self.cells = self._create_empty_grid()
    
    def is_valid_position(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position is within grid bounds.
        
        Args:
            pos: Position as (row, col)
            
        Returns:
            True if position is within bounds, False otherwise
        """
        row, col = pos
        return 0 <= row < self.height and 0 <= col < self.width
    
    def is_free(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position is free (not an obstacle).
        
        Args:
            pos: Position as (row, col)
            
        Returns:
            True if position is free, False if obstacle or out of bounds
        """
        if not self.is_valid_position(pos):
            return False
        row, col = pos
        return self.cells[row][col] == 0
    
    def is_obstacle(self, pos: Tuple[int, int]) -> bool:
        """
        Check if a position contains an obstacle.
        
        Args:
            pos: Position as (row, col)
            
        Returns:
            True if position has an obstacle, False otherwise
        """
        if not self.is_valid_position(pos):
            return False
        row, col = pos
        return self.cells[row][col] == 1
    
    def set_obstacle(self, pos: Tuple[int, int]) -> None:
        """
        Set a position as an obstacle.
        
        Args:
            pos: Position as (row, col)
        """
        if self.is_valid_position(pos):
            row, col = pos
            self.cells[row][col] = 1
    
    def set_free(self, pos: Tuple[int, int]) -> None:
        """
        Set a position as free (remove obstacle).
        
        Args:
            pos: Position as (row, col)
        """
        if self.is_valid_position(pos):
            row, col = pos
            self.cells[row][col] = 0
    
    def toggle_obstacle(self, pos: Tuple[int, int]) -> None:
        """
        Toggle obstacle state at a position.
        
        Args:
            pos: Position as (row, col)
        """
        if self.is_valid_position(pos):
            row, col = pos
            self.cells[row][col] = 1 - self.cells[row][col]  # Toggle between 0 and 1
    
    def get_obstacles(self) -> Set[Tuple[int, int]]:
        """
        Get all obstacle positions in the grid.
        
        Returns:
            Set of (row, col) tuples representing obstacle positions
        """
        obstacles = set()
        for row in range(self.height):
            for col in range(self.width):
                if self.cells[row][col] == 1:
                    obstacles.add((row, col))
        return obstacles
    
    def get_free_cells(self) -> List[Tuple[int, int]]:
        """
        Get all free cell positions in the grid.
        
        Returns:
            List of (row, col) tuples representing free cell positions
        """
        free_cells = []
        for row in range(self.height):
            for col in range(self.width):
                if self.cells[row][col] == 0:
                    free_cells.append((row, col))
        return free_cells
    
    def get_neighbors(self, pos: Tuple[int, int], 
                      include_diagonals: bool = False) -> List[Tuple[int, int]]:
        """
        Get neighboring free cells.
        
        Args:
            pos: Current position as (row, col)
            include_diagonals: Whether to include diagonal neighbors
            
        Returns:
            List of neighboring free cell positions
        """
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        if include_diagonals:
            directions.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])
        
        row, col = pos
        for dr, dc in directions:
            new_pos = (row + dr, col + dc)
            if self.is_free(new_pos):
                neighbors.append(new_pos)
        
        return neighbors
    
    def get_path_cost(self, path: List[Tuple[int, int]]) -> float:
        """
        Calculate total cost of a path.
        
        Assumes uniform cost of 1 per move.
        
        Args:
            path: List of positions forming the path
            
        Returns:
            Total path cost
        """
        if not path:
            return 0.0
        return len(path) - 1  # Number of moves
    
    def is_path_valid(self, path: List[Tuple[int, int]]) -> bool:
        """
        Check if a path is valid (all cells are free and connected).
        
        Args:
            path: List of positions forming the path
            
        Returns:
            True if path is valid, False otherwise
        """
        if not path:
            return False
        
        # Check all cells are free
        for pos in path:
            if not self.is_free(pos):
                return False
        
        # Check path is connected
        for i in range(len(path) - 1):
            if not self._are_adjacent(path[i], path[i + 1]):
                return False
        
        return True
    
    def _are_adjacent(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> bool:
        """
        Check if two positions are adjacent (4-connected).
        
        Args:
            pos1: First position
            pos2: Second position
            
        Returns:
            True if positions are adjacent, False otherwise
        """
        row1, col1 = pos1
        row2, col2 = pos2
        
        return abs(row1 - row2) + abs(col1 - col2) == 1
    
    def copy(self) -> 'Grid':
        """
        Create a deep copy of the grid.
        
        Returns:
            New Grid object with copied cells
        """
        new_grid = Grid(self.width, self.height)
        new_grid.cells = [row[:] for row in self.cells]
        return new_grid
    
    def to_dict(self) -> Dict:
        """
        Convert grid to dictionary representation.
        
        Returns:
            Dictionary with grid data
        """
        return {
            'width': self.width,
            'height': self.height,
            'cells': self.cells
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Grid':
        """
        Create grid from dictionary representation.
        
        Args:
            data: Dictionary with grid data
            
        Returns:
            New Grid object
        """
        grid = cls(data['width'], data['height'])
        grid.cells = data['cells']
        return grid
    
    def __str__(self) -> str:
        """String representation of the grid."""
        lines = []
        for row in self.cells:
            line = ''.join(['█' if cell == 1 else ' ' for cell in row])
            lines.append(line)
        return '\n'.join(lines)
    
    def __repr__(self) -> str:
        """Detailed representation of the grid."""
        return f"Grid(width={self.width}, height={self.height}, obstacles={len(self.get_obstacles())})"
