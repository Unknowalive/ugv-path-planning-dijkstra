"""
Visualization Utilities for UGV Path Planning.

This module provides console-based visualization for the grid,
path, and exploration process.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import os
import time
from typing import List, Tuple, Set, Optional, Dict


class GridVisualizer:
    """
    Console-based grid visualizer for UGV path planning.
    
    This class provides methods to visualize the grid environment,
    path, explored nodes, and obstacles in the console.
    
    Attributes:
        width: Grid width
        height: Grid height
        use_colors: Whether to use ANSI colors
    """
    
    # ANSI color codes (basic set for compatibility)
    COLORS = {
        'reset': '\033[0m',
        'bold': '\033[1m',
        'red': '\033[91m',
        'green': '\033[92m',
        'yellow': '\033[93m',
        'blue': '\033[94m',
        'cyan': '\033[96m',
        'gray': '\033[90m'
    }
    
    # Cell symbols - using ASCII-compatible characters
    SYMBOLS = {
        'free': ' ',
        'obstacle': '#',
        'path': '*',
        'explored': '.',
        'start': 'S',
        'goal': 'G',
        'ugv': 'U',
        'explored_path': 'o'
    }
    
    def __init__(self, width: int = 70, height: int = 70, use_colors: bool = True):
        """
        Initialize the visualizer.
        
        Args:
            width: Grid width
            height: Grid height
            use_colors: Whether to use ANSI colors
        """
        self.width = width
        self.height = height
        self.use_colors = use_colors and self._supports_colors()
    
    def _supports_colors(self) -> bool:
        """Check if terminal supports colors."""
        return hasattr(os, 'system') and os.environ.get('TERM') != 'dumb'
    
    def _colorize(self, text: str, color: str) -> str:
        """Add color to text if colors are enabled."""
        if self.use_colors and color in self.COLORS:
            return f"{self.COLORS[color]}{text}{self.COLORS['reset']}"
        return text
    
    def visualize(self, grid: List[List[int]],
                  obstacles: Optional[Set[Tuple[int, int]]] = None,
                  path: Optional[List[Tuple[int, int]]] = None,
                  explored: Optional[Set[Tuple[int, int]]] = None,
                  position: Optional[Tuple[int, int]] = None,
                  start: Optional[Tuple[int, int]] = None,
                  goal: Optional[Tuple[int, int]] = None,
                  show_legend: bool = True) -> str:
        """
        Create a visual representation of the grid.
        
        Args:
            grid: 2D grid (0=free, 1=obstacle)
            obstacles: Set of obstacle positions (overrides grid)
            path: List of positions forming the path
            explored: Set of explored node positions
            position: Current UGV position
            start: Start position
            goal: Goal position
            show_legend: Whether to show legend
            
        Returns:
            String representation of the visualization
        """
        lines = []
        
        # Header
        lines.append("=" * (self.width + 2))
        
        # Create position sets for quick lookup
        path_set = set(path) if path else set()
        explored_set = set(explored) if explored else set()
        obstacle_set = obstacles if obstacles else set()
        
        # Generate grid visualization
        for row in range(self.height):
            line = "|"
            
            for col in range(self.width):
                pos = (row, col)
                
                # Determine cell content
                cell = self._get_cell_char(
                    pos, grid, obstacle_set, path_set, explored_set,
                    position, start, goal
                )
                
                line += cell
            
            line += "|"
            lines.append(line)
        
        # Footer
        lines.append("=" * (self.width + 2))
        
        # Legend
        if show_legend:
            lines.append("")
            lines.append(self._create_legend(path, explored, position))
        
        return '\n'.join(lines)
    
    def _get_cell_char(self, pos: Tuple[int, int], grid: List[List[int]],
                       obstacles: Set[Tuple[int, int]], path: Set[Tuple[int, int]],
                       explored: Set[Tuple[int, int]], position: Optional[Tuple[int, int]],
                       start: Optional[Tuple[int, int]], goal: Optional[Tuple[int, int]]) -> str:
        """Get character for a cell."""
        row, col = pos
        
        # Check special positions first
        if position and pos == position:
            return self._colorize(self.SYMBOLS['ugv'], 'cyan')
        
        if start and pos == start:
            return self._colorize(self.SYMBOLS['start'], 'green')
        
        if goal and pos == goal:
            return self._colorize(self.SYMBOLS['goal'], 'red')
        
        # Check path
        if pos in path:
            return self._colorize(self.SYMBOLS['path'], 'yellow')
        
        # Check explored
        if pos in explored:
            return self._colorize(self.SYMBOLS['explored'], 'blue')
        
        # Check obstacle
        if pos in obstacles or (obstacles is None and row < len(grid) and col < len(grid[0]) and grid[row][col] == 1):
            return self._colorize(self.SYMBOLS['obstacle'], 'gray')
        
        # Free cell
        return self._colorize(self.SYMBOLS['free'], 'reset')
    
    def _create_legend(self, path: Optional[List], explored: Optional[Set],
                       position: Optional[Tuple]) -> str:
        """Create legend string."""
        legend_parts = []
        
        legend_parts.append(self._colorize(f" {self.SYMBOLS['start']} Start", 'green'))
        legend_parts.append(self._colorize(f" {self.SYMBOLS['goal']} Goal", 'red'))
        
        if position:
            legend_parts.append(self._colorize(f" {self.SYMBOLS['ugv']} UGV", 'cyan'))
        
        if path:
            legend_parts.append(self._colorize(f" {self.SYMBOLS['path']} Path", 'yellow'))
        
        if explored:
            legend_parts.append(self._colorize(f" {self.SYMBOLS['explored']} Explored", 'blue'))
        
        legend_parts.append(self._colorize(f" {self.SYMBOLS['obstacle']} Obstacle", 'gray'))
        legend_parts.append(self._colorize(f" {self.SYMBOLS['free']} Free", 'reset'))
        
        return "Legend: " + " | ".join(legend_parts)
    
    def visualize_step(self, grid: List[List[int]], step_info: Dict) -> None:
        """
        Visualize a single step of navigation.
        
        Args:
            grid: 2D grid
            step_info: Dictionary with step information
        """
        print(self.visualize(
            grid,
            obstacles=step_info.get('obstacles'),
            path=step_info.get('path'),
            explored=step_info.get('explored'),
            position=step_info.get('position'),
            start=step_info.get('start'),
            goal=step_info.get('goal')
        ))
        
        # Print status
        if 'message' in step_info:
            print(step_info['message'])
        
        print()
    
    def print_grid(self, grid: List[List[int]], 
                   title: str = "Grid Visualization") -> None:
        """
        Print a simple grid visualization.
        
        Args:
            grid: 2D grid
            title: Optional title
        """
        if title:
            print(f"\n{title}\n")
        
        # Create simple visualization
        lines = []
        for row in grid:
            line = ''.join(['#' if cell == 1 else ' ' for cell in row])
            lines.append(line)
        
        print('\n'.join(lines))


def print_path_info(path: List[Tuple[int, int]], cost: float, 
                    explored_count: int) -> None:
    """
    Print path information in a formatted way.
    
    Args:
        path: List of positions forming the path
        cost: Total path cost
        explored_count: Number of explored nodes
    """
    print("\n" + "=" * 50)
    print("PATH FOUND!")
    print("=" * 50)
    print(f"Path Length: {len(path)} steps")
    print(f"Total Cost: {cost}")
    print(f"Explored Nodes: {explored_count}")
    
    if path:
        print(f"\nStart: {path[0]}")
        print(f"Goal: {path[-1]}")
    
    print("=" * 50 + "\n")


def print_navigation_status(ugv, step: int) -> None:
    """
    Print current navigation status.
    
    Args:
        ugv: UGV object
        step: Current step number
    """
    status = ugv.get_status()
    
    print(f"Step {step}: ", end="")
    print(f"Position: {status['position']}, ", end="")
    print(f"Path remaining: {status['path_remaining']}, ", end="")
    print(f"At goal: {status['at_goal']}")


def create_ascii_animation(grid: List[List[int]],
                            path: List[Tuple[int, int]],
                            explored: Set[Tuple[int, int]],
                            start: Tuple[int, int],
                            goal: Tuple[int, int],
                            delay: float = 0.1) -> None:
    """
    Create an ASCII animation of the pathfinding process.
    
    Args:
        grid: 2D grid
        path: Path found
        explored: Explored nodes
        start: Start position
        goal: Goal position
        delay: Delay between frames
    """
    visualizer = GridVisualizer()
    
    # Show initial state with explored nodes
    print(visualizer.visualize(
        grid,
        path=path,
        explored=explored,
        start=start,
        goal=goal,
        show_legend=True
    ))
    
    time.sleep(delay * 2)
    
    # Animate UGV movement along path
    for i, pos in enumerate(path):
        # Clear screen (works on Windows and Unix)
        os.system('cls' if os.name == 'nt' else 'clear')
        
        remaining_path = path[i:]
        
        print(visualizer.visualize(
            grid,
            path=remaining_path,
            explored=explored,
            position=pos,
            start=start,
            goal=goal,
            show_legend=True
        ))
        
        print(f"\nMoving to step {i+1}/{len(path)}")
        
        time.sleep(delay)


def print_comparison(dijkstra_stats: Dict, astar_stats: Dict) -> None:
    """
    Print comparison between Dijkstra and A* algorithms.
    
    Args:
        dijkstra_stats: Statistics for Dijkstra
        astar_stats: Statistics for A*
    """
    print("\n" + "=" * 60)
    print("ALGORITHM COMPARISON")
    print("=" * 60)
    
    print(f"\n{'Metric':<25} {'Dijkstra':>15} {'A*':>15}")
    print("-" * 60)
    
    print(f"{'Path Length':<25} {dijkstra_stats.get('path_length', 0):>15} {astar_stats.get('path_length', 0):>15}")
    print(f"{'Total Cost':<25} {dijkstra_stats.get('cost', 0):>15.2f} {astar_stats.get('cost', 0):>15.2f}")
    print(f"{'Explored Nodes':<25} {dijkstra_stats.get('explored_count', 0):>15} {astar_stats.get('explored_count', 0):>15}")
    
    print("=" * 60 + "\n")


def visualize_city_graph(graph: Dict[str, Dict[str, float]]) -> None:
    """
    Visualize the city graph structure.
    
    Args:
        graph: City graph adjacency list
    """
    print("\n" + "=" * 50)
    print("CITY GRAPH STRUCTURE")
    print("=" * 50)
    
    for city, connections in sorted(graph.items()):
        print(f"\n{city}:")
        for neighbor, distance in sorted(connections.items()):
            print(f"  -> {neighbor}: {distance} km")
    
    print("\n" + "=" * 50 + "\n")
