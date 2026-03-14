"""
A* (A-Star) Algorithm Implementation for Path Planning.

This module provides an A* algorithm implementation for grid-based
path planning. A* is an informed search algorithm that uses heuristics
to guide its search, making it more efficient than Dijkstra's algorithm
for finding paths in grids.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import heapq
from typing import Dict, List, Tuple, Set, Optional


class AStarNavigator:
    """
    A* algorithm implementation for grid-based path planning.
    
    A* combines the actual cost from the start (g(n)) with a heuristic
    estimate to the goal (h(n)) to prioritize exploring nodes that are
    more likely to lead to the goal.
    
    Attributes:
        grid_width: Width of the grid
        grid_height: Height of the grid
        move_cost: Cost to move to an adjacent cell
    """
    
    def __init__(self, grid_width: int = 70, grid_height: int = 70,
                 move_cost: float = 1.0, allow_diagonal: bool = False):
        """
        Initialize the A* navigator.
        
        Args:
            grid_width: Width of the grid
            grid_height: Height of the grid
            move_cost: Cost to move to an adjacent cell
            allow_diagonal: Whether to allow diagonal movement
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.move_cost = move_cost
        self.allow_diagonal = allow_diagonal
    
    def find_path(self, grid: List[List[int]],
                  start: Tuple[int, int],
                  goal: Tuple[int, int],
                  track_explored: bool = True) -> Tuple[Optional[List[Tuple[int, int]]], float, Set[Tuple[int, int]]]:
        """
        Find the shortest path in a grid using A* algorithm.
        
        Args:
            grid: 2D grid where 0 represents free cell and 1 represents obstacle
            start: Starting position as (row, col)
            goal: Goal position as (row, col)
            track_explored: Whether to track explored nodes
            
        Returns:
            Tuple of (path as list of positions, total cost, explored nodes set)
            Returns (None, float('inf'), set()) if no path exists
        """
        # Validate start and goal positions
        if not self._is_valid_position(start, grid):
            raise ValueError(f"Invalid start position: {start}")
        if not self._is_valid_position(goal, grid):
            raise ValueError(f"Invalid goal position: {goal}")
        
        # Handle trivial case
        if start == goal:
            return [start], 0.0, set()
        
        # Priority queue: (f_score, g_score, position, path)
        # f_score = g_score + h_score (heuristic)
        # Using g_score as tiebreaker for consistent ordering
        start_heuristic = self._heuristic(start, goal)
        pq = [(start_heuristic, 0.0, start, [start])]
        
        # Track g_score (actual cost from start) for each position
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}
        
        # Set of visited positions
        visited: Set[Tuple[int, int]] = set()
        
        # Track explored nodes for visualization
        explored: Set[Tuple[int, int]] = set()
        
        # Get movement directions
        directions = self._get_directions()
        
        while pq:
            # Get position with minimum f_score
            f_score, current_g, current_pos, path = heapq.heappop(pq)
            
            # Skip if already visited
            if current_pos in visited:
                continue
            
            # Mark as visited
            visited.add(current_pos)
            
            # Track explored nodes (excluding start and goal)
            if track_explored and current_pos != start and current_pos != goal:
                explored.add(current_pos)
            
            # Check if we reached the goal
            if current_pos == goal:
                return path, current_g, explored
            
            # Explore neighbors
            row, col = current_pos
            for dr, dc in directions:
                new_row, new_col = row + dr, col + dc
                new_pos = (new_row, new_col)
                
                # Check if valid position
                if not self._is_valid_position(new_pos, grid):
                    continue
                
                # Skip if already visited
                if new_pos in visited:
                    continue
                
                # Calculate new g_score
                # Add diagonal cost if moving diagonally
                if dr != 0 and dc != 0:
                    new_g = current_g + self.move_cost * 1.414  # sqrt(2)
                else:
                    new_g = current_g + self.move_cost
                
                # Update if we found a shorter path
                if new_pos not in g_score or new_g < g_score[new_pos]:
                    g_score[new_pos] = new_g
                    
                    # Calculate f_score = g_score + heuristic
                    h_score = self._heuristic(new_pos, goal)
                    new_f = new_g + h_score
                    
                    new_path = path + [new_pos]
                    heapq.heappush(pq, (new_f, new_g, new_pos, new_path))
        
        # No path found
        return None, float('inf'), explored
    
    def _heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """
        Calculate heuristic cost using Manhattan distance.
        
        The Manhattan distance is used for 4-directional movement,
        which is admissible (never overestimates the true cost).
        
        Args:
            pos: Current position
            goal: Goal position
            
        Returns:
            Heuristic cost estimate
        """
        # Manhattan distance for 4-directional movement
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def _is_valid_position(self, pos: Tuple[int, int], grid: List[List[int]]) -> bool:
        """
        Check if a position is valid (within bounds and not an obstacle).
        
        Args:
            pos: Position as (row, col)
            grid: 2D grid
            
        Returns:
            True if position is valid, False otherwise
        """
        row, col = pos
        
        # Check bounds
        if row < 0 or row >= len(grid) or col < 0 or col >= len(grid[0]):
            return False
        
        # Check if not an obstacle
        if grid[row][col] == 1:
            return False
        
        return True
    
    def _get_directions(self) -> List[Tuple[int, int]]:
        """
        Get movement directions based on configuration.
        
        Returns:
            List of (row_delta, col_delta) tuples
        """
        if self.allow_diagonal:
            # 8-directional movement including diagonals
            return [(-1, 0), (1, 0), (0, -1), (0, 1),
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]
        else:
            # 4-directional movement only
            return [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    def get_neighbors(self, pos: Tuple[int, int],
                       grid: List[List[int]]) -> List[Tuple[Tuple[int, int], float]]:
        """
        Get valid neighbors of a position.
        
        Args:
            pos: Current position as (row, col)
            grid: 2D grid
            
        Returns:
            List of (neighbor_position, cost) tuples
        """
        neighbors = []
        directions = self._get_directions()
        
        row, col = pos
        for dr, dc in directions:
            new_pos = (row + dr, col + dc)
            if self._is_valid_position(new_pos, grid):
                # Calculate cost
                if dr != 0 and dc != 0:
                    cost = self.move_cost * 1.414
                else:
                    cost = self.move_cost
                neighbors.append((new_pos, cost))
        
        return neighbors


class AStarWithDynamicReplanning(AStarNavigator):
    """
    A* algorithm with support for dynamic replanning.
    
    This class extends AStarNavigator with additional methods
    for handling dynamic obstacle changes during navigation.
    """
    
    def find_path_avoiding(self, grid: List[List[int]],
                           start: Tuple[int, int],
                           goal: Tuple[int, int],
                           blocked_positions: Set[Tuple[int, int]],
                           track_explored: bool = True) -> Tuple[Optional[List[Tuple[int, int]]], float, Set[Tuple[int, int]]]:
        """
        Find path while avoiding specific blocked positions.
        
        This is useful for dynamic replanning when obstacles appear
        along the current path.
        
        Args:
            grid: 2D grid
            start: Starting position
            goal: Goal position
            blocked_positions: Set of positions that should be treated as obstacles
            track_explored: Whether to track explored nodes
            
        Returns:
            Tuple of (path, cost, explored nodes)
        """
        # Create a modified grid that treats blocked positions as obstacles
        modified_grid = [row[:] for row in grid]  # Make a copy
        
        for pos in blocked_positions:
            row, col = pos
            if 0 <= row < len(modified_grid) and 0 <= col < len(modified_grid[0]):
                modified_grid[row][col] = 1
        
        # Use parent class method
        return self.find_path(modified_grid, start, goal, track_explored)
    
    def replan_from_position(self, grid: List[List[int]],
                               current_pos: Tuple[int, int],
                               goal: Tuple[int, int],
                               blocked_path: List[Tuple[int, int]],
                               track_explored: bool = True) -> Tuple[Optional[List[Tuple[int, int]]], float, Set[Tuple[int, int]]]:
        """
        Replan path from current position avoiding blocked path.
        
        This is called when an obstacle appears on the current path.
        
        Args:
            grid: 2D grid
            current_pos: Current UGV position
            goal: Goal position
            blocked_path: Portion of path that is now blocked
            track_explored: Whether to track explored nodes
            
        Returns:
            Tuple of (new_path, cost, explored nodes)
        """
        # Treat the blocked path as obstacles
        blocked_set = set(blocked_path)
        
        return self.find_path_avoiding(grid, current_pos, goal, blocked_set, track_explored)


def compare_algorithms(grid: List[List[int]],
                        start: Tuple[int, int],
                        goal: Tuple[int, int]) -> Dict[str, Dict[str, any]]:
    """
    Compare Dijkstra and A* algorithms on the same grid.
    
    Args:
        grid: 2D grid
        start: Starting position
        goal: Goal position
        
    Returns:
        Dictionary with comparison results
    """
    from algorithms.dijkstra import DijkstraGridNavigator
    
    # Test with Dijkstra
    dijkstra = DijkstraGridNavigator()
    dijkstra_path, dijkstra_cost, dijkstra_explored = dijkstra.find_path(
        grid, start, goal, track_explored=True
    )
    
    # Test with A*
    astar = AStarNavigator()
    astar_path, astar_cost, astar_explored = astar.find_path(
        grid, start, goal, track_explored=True
    )
    
    return {
        'dijkstra': {
            'path': dijkstra_path,
            'cost': dijkstra_cost,
            'explored_count': len(dijkstra_explored),
            'path_length': len(dijkstra_path) if dijkstra_path else 0
        },
        'astar': {
            'path': astar_path,
            'cost': astar_cost,
            'explored_count': len(astar_explored),
            'path_length': len(astar_path) if astar_path else 0
        }
    }
