"""
Dijkstra's Algorithm Implementation for Path Planning.

This module provides implementations of Dijkstra's algorithm for both:
1. City navigation (graph-based with weighted edges)
2. UGV grid navigation (2D grid with obstacles)

Dijkstra's algorithm finds the shortest path between nodes in a weighted
graph. It uses a priority queue to explore nodes in order of their
distance from the source.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import heapq
from typing import Dict, List, Tuple, Optional, Set, Any
from collections import defaultdict


class DijkstraCityNavigator:
    """
    Dijkstra's algorithm implementation for city navigation.
    
    This class implements Dijkstra's algorithm to find the shortest path
    between cities in a weighted graph where edges represent roads with
    distances as weights.
    
    Attributes:
        graph: Dictionary representing adjacency list of cities
    """
    
    def __init__(self, graph: Optional[Dict[str, Dict[str, float]]] = None):
        """
        Initialize the city navigator with a graph.
        
        Args:
            graph: Adjacency list representing cities and roads.
                  Format: {city: {neighbor_city: distance, ...}, ...}
        """
        self.graph = graph if graph is not None else {}
    
    def add_city(self, city: str) -> None:
        """
        Add a new city to the graph.
        
        Args:
            city: Name of the city to add
        """
        if city not in self.graph:
            self.graph[city] = {}
    
    def add_road(self, city1: str, city2: str, distance: float) -> None:
        """
        Add a road (edge) between two cities.
        
        Args:
            city1: First city
            city2: Second city
            distance: Distance/weight of the road
        """
        # Add cities if they don't exist
        self.add_city(city1)
        self.add_city(city2)
        
        # Add bidirectional roads
        self.graph[city1][city2] = distance
        self.graph[city2][city1] = distance
    
    def find_shortest_path(self, start: str, goal: str) -> Tuple[Optional[List[str]], float]:
        """
        Find the shortest path between two cities using Dijkstra's algorithm.
        
        Args:
            start: Starting city name
            goal: Goal city name
            
        Returns:
            Tuple of (path as list of cities, total distance)
            Returns (None, float('inf')) if no path exists
        """
        # Validate start and goal cities exist
        if start not in self.graph:
            raise ValueError(f"Start city '{start}' not found in graph")
        if goal not in self.graph:
            raise ValueError(f"Goal city '{goal}' not found in graph")
        
        # Handle trivial case
        if start == goal:
            return [start], 0.0
        
        # Priority queue: (distance, city, path)
        # Using city name as tiebreaker for consistent ordering
        pq = [(0.0, start, [start])]
        
        # Dictionary to track minimum distance to each city
        min_distance: Dict[str, float] = {start: 0.0}
        
        # Set of visited cities
        visited: Set[str] = set()
        
        while pq:
            # Get city with minimum distance
            current_dist, current_city, path = heapq.heappop(pq)
            
            # Skip if already visited
            if current_city in visited:
                continue
            
            # Mark as visited
            visited.add(current_city)
            
            # Check if we reached the goal
            if current_city == goal:
                return path, current_dist
            
            # Explore neighbors
            for neighbor, weight in self.graph[current_city].items():
                if neighbor in visited:
                    continue
                
                # Calculate new distance
                new_dist = current_dist + weight
                
                # Update if we found a shorter path
                if neighbor not in min_distance or new_dist < min_distance[neighbor]:
                    min_distance[neighbor] = new_dist
                    new_path = path + [neighbor]
                    heapq.heappush(pq, (new_dist, neighbor, new_path))
        
        # No path found
        return None, float('inf')
    
    def get_all_distances(self, start: str) -> Dict[str, float]:
        """
        Calculate shortest distances from start city to all reachable cities.
        
        Args:
            start: Starting city name
            
        Returns:
            Dictionary mapping city names to their shortest distances from start
        """
        if start not in self.graph:
            raise ValueError(f"Start city '{start}' not found in graph")
        
        # Priority queue: (distance, city)
        pq = [(0.0, start)]
        
        # Track minimum distances
        min_distance: Dict[str, float] = {start: 0.0}
        
        # Set of visited cities
        visited: Set[str] = set()
        
        while pq:
            current_dist, current_city = heapq.heappop(pq)
            
            if current_city in visited:
                continue
            
            visited.add(current_city)
            
            # Explore neighbors
            for neighbor, weight in self.graph[current_city].items():
                if neighbor in visited:
                    continue
                
                new_dist = current_dist + weight
                
                if neighbor not in min_distance or new_dist < min_distance[neighbor]:
                    min_distance[neighbor] = new_dist
                    heapq.heappush(pq, (new_dist, neighbor))
        
        return min_distance


class DijkstraGridNavigator:
    """
    Dijkstra's algorithm implementation for grid-based path planning.
    
    This class implements Dijkstra's algorithm for finding the shortest
    path in a 2D grid with obstacles. It supports 4-directional movement
    (up, down, left, right).
    
    Attributes:
        grid_width: Width of the grid
        grid_height: Height of the grid
        move_cost: Cost to move to an adjacent cell
    """
    
    def __init__(self, grid_width: int = 70, grid_height: int = 70, 
                 move_cost: float = 1.0):
        """
        Initialize the grid navigator.
        
        Args:
            grid_width: Width of the grid
            grid_height: Height of the grid
            move_cost: Cost to move to an adjacent cell
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.move_cost = move_cost
    
    def find_path(self, grid: List[List[int]], 
                  start: Tuple[int, int], 
                  goal: Tuple[int, int],
                  track_explored: bool = True) -> Tuple[Optional[List[Tuple[int, int]]], float, Set[Tuple[int, int]]]:
        """
        Find the shortest path in a grid using Dijkstra's algorithm.
        
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
        
        # Priority queue: (cost, position, path)
        pq = [(0.0, start, [start])]
        
        # Track minimum cost to reach each position
        min_cost: Dict[Tuple[int, int], float] = {start: 0.0}
        
        # Set of visited positions
        visited: Set[Tuple[int, int]] = set()
        
        # Track explored nodes for visualization
        explored: Set[Tuple[int, int]] = set()
        
        # 4-directional movement: up, down, left, right
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        while pq:
            # Get position with minimum cost
            current_cost, current_pos, path = heapq.heappop(pq)
            
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
                return path, current_cost, explored
            
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
                
                # Calculate new cost
                new_cost = current_cost + self.move_cost
                
                # Update if we found a shorter path
                if new_pos not in min_cost or new_cost < min_cost[new_pos]:
                    min_cost[new_pos] = new_cost
                    new_path = path + [new_pos]
                    heapq.heappush(pq, (new_cost, new_pos, new_path))
        
        # No path found
        return None, float('inf'), explored
    
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
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        row, col = pos
        for dr, dc in directions:
            new_pos = (row + dr, col + dc)
            if self._is_valid_position(new_pos, grid):
                neighbors.append((new_pos, self.move_cost))
        
        return neighbors


def create_sample_city_graph() -> Dict[str, Dict[str, float]]:
    """
    Create a sample city graph for demonstration.
    
    Returns:
        Dictionary representing adjacency list of Indian cities
    """
    return {
        'Delhi': {'Mumbai': 1157, 'Kolkata': 1471, 'Chennai': 2201, 'Bangalore': 2150, 'Jaipur': 280},
        'Mumbai': {'Delhi': 1157, 'Bangalore': 983, 'Chennai': 1334, 'Kolkata': 1677, 'Pune': 150},
        'Bangalore': {'Mumbai': 983, 'Delhi': 2150, 'Chennai': 350, 'Kolkata': 1567, 'Hyderabad': 570},
        'Chennai': {'Bangalore': 350, 'Mumbai': 1334, 'Delhi': 2201, 'Kolkata': 1683, 'Hyderabad': 630},
        'Kolkata': {'Delhi': 1471, 'Mumbai': 1677, 'Bangalore': 1567, 'Chennai': 1683},
        'Hyderabad': {'Bangalore': 570, 'Mumbai': 660, 'Chennai': 630, 'Delhi': 1250, 'Pune': 560},
        'Ahmedabad': {'Mumbai': 530, 'Delhi': 915, 'Jaipur': 295, 'Surat': 270},
        'Jaipur': {'Delhi': 280, 'Ahmedabad': 295, 'Kolkata': 1450, 'Mumbai': 1170},
        'Surat': {'Mumbai': 260, 'Ahmedabad': 270, 'Bangalore': 1160, 'Hyderabad': 760},
        'Pune': {'Mumbai': 150, 'Bangalore': 840, 'Hyderabad': 560, 'Delhi': 1400}
    }
