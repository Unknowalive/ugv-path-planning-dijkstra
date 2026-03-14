"""
Main Entry Point for UGV Path Planning Project.

This module provides the main interface for running the UGV path
planning system with Dijkstra and A* algorithms.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import sys
import argparse
from typing import Optional

from config import (
    GRID_WIDTH, GRID_HEIGHT, DEFAULT_OBSTACLE_DENSITY,
    DEFAULT_ALGORITHM, OBSTACLE_DENSITY
)
from environment.grid import Grid
from environment.obstacle_generator import ObstacleGenerator, generate_grid_with_obstacles
from algorithms.dijkstra import DijkstraCityNavigator, DijkstraGridNavigator, create_sample_city_graph
from algorithms.astar import AStarNavigator
from simulation.ugv import UGV, UGVSsimulator
from simulation.dynamic_replanning import DynamicReplanningNavigator, demonstrate_dynamic_replanning
from utils.visualization import (
    GridVisualizer, print_path_info, print_comparison,
    visualize_city_graph
)


def run_city_navigation(city_from: str = "Delhi", city_to: str = "Mumbai") -> None:
    """
    Run city navigation demonstration using Dijkstra's algorithm.
    
    Args:
        city_from: Starting city name
        city_to: Goal city name
    """
    print("\n" + "=" * 60)
    print("PART 1: CITY NAVIGATION USING DIJKSTRA'S ALGORITHM")
    print("=" * 60)
    
    # Create city graph
    graph = create_sample_city_graph()
    
    # Visualize graph
    visualize_city_graph(graph)
    
    # Create navigator
    navigator = DijkstraCityNavigator(graph)
    
    # Find shortest path
    print(f"\nFinding shortest path from {city_from} to {city_to}...")
    path, distance = navigator.find_shortest_path(city_from, city_to)
    
    if path:
        print(f"\nShortest Path: {' -> '.join(path)}")
        print(f"Total Distance: {distance} km")
        
        # Show individual legs
        print("\nJourney Details:")
        for i in range(len(path) - 1):
            leg_distance = graph[path[i]][path[i + 1]]
            print(f"  {path[i]} -> {path[i + 1]}: {leg_distance} km")
    else:
        print("No path found!")
    
    # Get all distances from source
    print(f"\nAll distances from {city_from}:")
    distances = navigator.get_all_distances(city_from)
    for city, dist in sorted(distances.items(), key=lambda x: x[1]):
        print(f"  {city}: {dist} km")


def run_ugv_navigation(grid_size: tuple = (20, 20),
                       density: str = 'medium',
                       algorithm: str = 'dijkstra',
                       visualize: bool = True) -> None:
    """
    Run UGV grid navigation demonstration.
    
    Args:
        grid_size: Size of the grid (width, height)
        density: Obstacle density level
        algorithm: Pathfinding algorithm
        visualize: Whether to visualize the result
    """
    print("\n" + "=" * 60)
    print("PART 2: UGV NAVIGATION IN GRID ENVIRONMENT")
    print("=" * 60)
    
    print(f"\nConfiguration:")
    print(f"  Grid Size: {grid_size[0]}x{grid_size[1]}")
    print(f"  Obstacle Density: {density} ({OBSTACLE_DENSITY[density]*100}%)")
    print(f"  Algorithm: {algorithm}")
    
    # Generate grid with obstacles
    print(f"\nGenerating grid with {density} density obstacles...")
    grid = generate_grid_with_obstacles(
        grid_size[0], grid_size[1],
        density=density,
        random_seed=42,
        ensure_connectivity=True
    )
    
    # Define start and goal
    start = (0, 0)
    goal = (grid_size[1] - 1, grid_size[0] - 1)
    
    print(f"  Start: {start}")
    print(f"  Goal: {goal}")
    print(f"  Obstacles: {len(grid.get_obstacles())}")
    
    # Create UGV and find path
    print(f"\nFinding path using {algorithm}...")
    ugv = UGV(start, goal, grid, algorithm)
    path = ugv.find_path()
    
    if path:
        status = ugv.get_status()
        
        print(f"\nPath found!")
        print(f"  Path Length: {len(path)} steps")
        print(f"  Total Cost: {status['total_cost']}")
        print(f"  Explored Nodes: {status['explored_count']}")
        
        # Visualize if requested
        if visualize:
            visualizer = GridVisualizer()
            
            print("\nGrid Visualization:")
            print(visualizer.visualize(
                grid.cells,
                obstacles=grid.get_obstacles(),
                path=path,
                explored=ugv.explored_nodes,
                start=start,
                goal=goal
            ))
    else:
        print("No path found! Try with a lower obstacle density.")


def run_dynamic_obstacles(grid_size: tuple = (20, 20),
                          algorithm: str = 'dijkstra') -> None:
    """
    Run dynamic obstacle replanning demonstration.
    
    Args:
        grid_size: Size of the grid
        algorithm: Pathfinding algorithm
    """
    print("\n" + "=" * 60)
    print("PART 3: DYNAMIC OBSTACLES AND REPLANNING")
    print("=" * 60)
    
    print(f"\nConfiguration:")
    print(f"  Grid Size: {grid_size[0]}x{grid_size[1]}")
    print(f"  Algorithm: {algorithm}")
    
    # Run demonstration
    success = demonstrate_dynamic_replanning(
        grid_size=grid_size,
        start=(0, 0),
        goal=(grid_size[1] - 1, grid_size[0] - 1),
        algorithm=algorithm,
        obstacle_density=0.15,
        obstacle_probability=0.15,
        verbose=True
    )
    
    if success:
        print("\nUGV successfully reached the goal with dynamic replanning!")
    else:
        print("\nUGV could not reach the goal.")


def compare_algorithms(grid_size: tuple = (20, 20), density: str = 'medium') -> None:
    """
    Compare Dijkstra and A* algorithms.
    
    Args:
        grid_size: Size of the grid
        density: Obstacle density
    """
    print("\n" + "=" * 60)
    print("ALGORITHM COMPARISON: DIJKSTRA vs A*")
    print("=" * 60)
    
    # Generate grid
    grid = generate_grid_with_obstacles(
        grid_size[0], grid_size[1],
        density=density,
        random_seed=42,
        ensure_connectivity=True
    )
    
    start = (0, 0)
    goal = (grid_size[1] - 1, grid_size[0] - 1)
    
    # Test Dijkstra
    print("\nRunning Dijkstra's Algorithm...")
    dijkstra = DijkstraGridNavigator()
    dijkstra_path, dijkstra_cost, dijkstra_explored = dijkstra.find_path(
        grid.cells, start, goal
    )
    
    # Test A*
    print("Running A* Algorithm...")
    astar = AStarNavigator()
    astar_path, astar_cost, astar_explored = astar.find_path(
        grid.cells, start, goal
    )
    
    # Compare results
    print_comparison(
        {
            'path_length': len(dijkstra_path) if dijkstra_path else 0,
            'cost': dijkstra_cost,
            'explored_count': len(dijkstra_explored)
        },
        {
            'path_length': len(astar_path) if astar_path else 0,
            'cost': astar_cost,
            'explored_count': len(astar_explored)
        }
    )
    
    # Visualize A* result
    if astar_path:
        visualizer = GridVisualizer()
        print("A* Path Visualization:")
        print(visualizer.visualize(
            grid.cells,
            obstacles=grid.get_obstacles(),
            path=astar_path,
            explored=astar_explored,
            start=start,
            goal=goal
        ))


def run_tests() -> None:
    """Run unit tests."""
    print("\n" + "=" * 60)
    print("RUNNING UNIT TESTS")
    print("=" * 60 + "\n")
    
    from tests.test_dijkstra import run_tests
    run_tests()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='UGV Path Planning using Dijkstra and A* Algorithms'
    )
    
    parser.add_argument(
        '--mode', '-m',
        choices=['city', 'ugv', 'dynamic', 'compare', 'test', 'all'],
        default='all',
        help='Mode to run'
    )
    
    parser.add_argument(
        '--algorithm', '-a',
        choices=['dijkstra', 'astar'],
        default='dijkstra',
        help='Pathfinding algorithm'
    )
    
    parser.add_argument(
        '--density', '-d',
        choices=['low', 'medium', 'high'],
        default='medium',
        help='Obstacle density'
    )
    
    parser.add_argument(
        '--grid-size', '-g',
        type=int,
        nargs=2,
        default=[20, 20],
        help='Grid size (width height)'
    )
    
    parser.add_argument(
        '--from', '-f',
        dest='city_from',
        default='Delhi',
        help='Starting city for navigation'
    )
    
    parser.add_argument(
        '--to', '-t',
        dest='city_to',
        default='Mumbai',
        help='Goal city for navigation'
    )
    
    parser.add_argument(
        '--no-visualize',
        action='store_true',
        help='Disable visualization'
    )
    
    args = parser.parse_args()
    
    print("\n" + "=" * 60)
    print("UGV PATH PLANNING SYSTEM")
    print("Dijkstra and A* Algorithms Implementation")
    print("=" * 60)
    
    # Run selected mode
    if args.mode == 'city' or args.mode == 'all':
        run_city_navigation(args.city_from, args.city_to)
    
    if args.mode == 'ugv' or args.mode == 'all':
        run_ugv_navigation(
            tuple(args.grid_size),
            args.density,
            args.algorithm,
            not args.no_visualize
        )
    
    if args.mode == 'dynamic' or args.mode == 'all':
        run_dynamic_obstacles(tuple(args.grid_size), args.algorithm)
    
    if args.mode == 'compare':
        compare_algorithms(tuple(args.grid_size), args.density)
    
    if args.mode == 'test':
        run_tests()
    
    print("\n" + "=" * 60)
    print("Program execution completed!")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    main()
