"""
Unit Tests for Dijkstra's Algorithm Implementation.

This module contains unit tests for the Dijkstra algorithm
implementations in both city navigation and grid navigation modes.

Author: UGV Path Planning Team
Version: 1.0.0
"""

import unittest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algorithms.dijkstra import DijkstraCityNavigator, DijkstraGridNavigator, create_sample_city_graph
from environment.grid import Grid
from environment.obstacle_generator import ObstacleGenerator


class TestDijkstraCityNavigator(unittest.TestCase):
    """Test cases for Dijkstra city navigation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.graph = create_sample_city_graph()
        self.navigator = DijkstraCityNavigator(self.graph)
    
    def test_shortest_path_delhi_to_mumbai(self):
        """Test shortest path from Delhi to Mumbai."""
        path, distance = self.navigator.find_shortest_path('Delhi', 'Mumbai')
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], 'Delhi')
        self.assertEqual(path[-1], 'Mumbai')
        self.assertLessEqual(distance, 1157)  # Direct route
    
    def test_shortest_path_delhi_to_chennai(self):
        """Test shortest path from Delhi to Chennai."""
        path, distance = self.navigator.find_shortest_path('Delhi', 'Chennai')
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], 'Delhi')
        self.assertEqual(path[-1], 'Chennai')
    
    def test_same_city_path(self):
        """Test path from a city to itself."""
        path, distance = self.navigator.find_shortest_path('Delhi', 'Delhi')
        
        self.assertEqual(path, ['Delhi'])
        self.assertEqual(distance, 0.0)
    
    def test_path_not_found_invalid_city(self):
        """Test path with invalid city."""
        with self.assertRaises(ValueError):
            self.navigator.find_shortest_path('Delhi', 'InvalidCity')
    
    def test_get_all_distances(self):
        """Test getting all distances from a city."""
        distances = self.navigator.get_all_distances('Delhi')
        
        self.assertIn('Mumbai', distances)
        self.assertIn('Bangalore', distances)
        self.assertEqual(distances['Delhi'], 0.0)
    
    def test_add_city(self):
        """Test adding a new city."""
        new_navigator = DijkstraCityNavigator()
        new_navigator.add_city('NewCity')
        
        self.assertIn('NewCity', new_navigator.graph)
    
    def test_add_road(self):
        """Test adding a road between cities."""
        new_navigator = DijkstraCityNavigator()
        new_navigator.add_road('CityA', 'CityB', 100)
        
        self.assertIn('CityA', new_navigator.graph)
        self.assertIn('CityB', new_navigator.graph)
        self.assertEqual(new_navigator.graph['CityA']['CityB'], 100)
        self.assertEqual(new_navigator.graph['CityB']['CityA'], 100)


class TestDijkstraGridNavigator(unittest.TestCase):
    """Test cases for Dijkstra grid navigation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.navigator = DijkstraGridNavigator(grid_width=10, grid_height=10)
        self.grid = [[0] * 10 for _ in range(10)]
    
    def test_simple_path(self):
        """Test simple path in empty grid."""
        path, cost, explored = self.navigator.find_path(
            self.grid, (0, 0), (9, 9)
        )
        
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (9, 9))
        self.assertEqual(len(explored) > 0, True)
    
    def test_same_position(self):
        """Test path from same position to itself."""
        path, cost, explored = self.navigator.find_path(
            self.grid, (5, 5), (5, 5)
        )
        
        self.assertEqual(path, [(5, 5)])
        self.assertEqual(cost, 0.0)
    
    def test_path_with_obstacle(self):
        """Test path with obstacles."""
        # Add a wall blocking direct path
        for i in range(5):
            self.grid[5][i] = 1
        
        path, cost, explored = self.navigator.find_path(
            self.grid, (0, 0), (9, 9)
        )
        
        # Path should exist but go around
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (9, 9))
    
    def test_no_path_exists(self):
        """Test when no path exists due to complete blockage."""
        # Create a wall that completely blocks the grid
        for i in range(10):
            self.grid[i][5] = 1
        
        path, cost, explored = self.navigator.find_path(
            self.grid, (0, 0), (9, 9)
        )
        
        self.assertIsNone(path)
        self.assertEqual(cost, float('inf'))
    
    def test_invalid_start(self):
        """Test with invalid start position."""
        with self.assertRaises(ValueError):
            self.navigator.find_path(self.grid, (-1, 0), (9, 9))
    
    def test_invalid_goal(self):
        """Test with invalid goal position."""
        with self.assertRaises(ValueError):
            self.navigator.find_path(self.grid, (0, 0), (10, 10))
    
    def test_obstacle_at_goal(self):
        """Test when goal is blocked by obstacle."""
        self.grid[9][9] = 1
        
        with self.assertRaises(ValueError):
            self.navigator.find_path(self.grid, (0, 0), (9, 9))
    
    def test_get_neighbors(self):
        """Test getting neighbors of a position."""
        neighbors = self.navigator.get_neighbors((5, 5), self.grid)
        
        # Should have 4 neighbors in 10x10 grid
        self.assertEqual(len(neighbors), 4)


class TestGrid(unittest.TestCase):
    """Test cases for Grid class."""
    
    def test_grid_creation(self):
        """Test grid creation with default values."""
        grid = Grid(10, 10)
        
        self.assertEqual(grid.width, 10)
        self.assertEqual(grid.height, 10)
        self.assertEqual(len(grid.cells), 10)
        self.assertEqual(len(grid.cells[0]), 10)
    
    def test_is_valid_position(self):
        """Test position validation."""
        grid = Grid(10, 10)
        
        self.assertTrue(grid.is_valid_position((0, 0)))
        self.assertTrue(grid.is_valid_position((9, 9)))
        self.assertFalse(grid.is_valid_position((-1, 0)))
        self.assertFalse(grid.is_valid_position((0, 10)))
        self.assertFalse(grid.is_valid_position((10, 10)))
    
    def test_obstacle_operations(self):
        """Test setting and checking obstacles."""
        grid = Grid(10, 10)
        
        # Set obstacle
        grid.set_obstacle((5, 5))
        self.assertTrue(grid.is_obstacle((5, 5)))
        self.assertFalse(grid.is_free((5, 5)))
        
        # Set free
        grid.set_free((5, 5))
        self.assertFalse(grid.is_obstacle((5, 5)))
        self.assertTrue(grid.is_free((5, 5)))
    
    def test_get_obstacles(self):
        """Test getting all obstacles."""
        grid = Grid(10, 10)
        
        grid.set_obstacle((1, 1))
        grid.set_obstacle((2, 2))
        grid.set_obstacle((3, 3))
        
        obstacles = grid.get_obstacles()
        
        self.assertEqual(len(obstacles), 3)
        self.assertIn((1, 1), obstacles)
        self.assertIn((2, 2), obstacles)
        self.assertIn((3, 3), obstacles)
    
    def test_get_neighbors(self):
        """Test getting neighboring cells."""
        grid = Grid(10, 10)
        
        neighbors = grid.get_neighbors((5, 5))
        
        # Should have 4 neighbors
        self.assertEqual(len(neighbors), 4)
        self.assertIn((4, 5), neighbors)
        self.assertIn((6, 5), neighbors)
        self.assertIn((5, 4), neighbors)
        self.assertIn((5, 6), neighbors)
    
    def test_is_path_valid(self):
        """Test path validation."""
        grid = Grid(10, 10)
        
        # Valid path
        path = [(0, 0), (0, 1), (1, 1)]
        self.assertTrue(grid.is_path_valid(path))
        
        # Path with obstacle
        grid.set_obstacle((0, 1))
        self.assertFalse(grid.is_path_valid(path))
    
    def test_grid_copy(self):
        """Test grid copying."""
        grid = Grid(10, 10)
        grid.set_obstacle((5, 5))
        
        grid_copy = grid.copy()
        
        # Modify original
        grid.set_obstacle((6, 6))
        
        # Copy should be unchanged
        self.assertFalse(grid_copy.is_obstacle((6, 6)))
        self.assertTrue(grid_copy.is_obstacle((5, 5)))


class TestObstacleGenerator(unittest.TestCase):
    """Test cases for ObstacleGenerator class."""
    
    def test_generate_random(self):
        """Test random obstacle generation."""
        grid = Grid(10, 10)
        generator = ObstacleGenerator(grid, random_seed=42)
        
        obstacles = generator.generate_random(density=0.2, ensure_connectivity=False)
        
        # Should have approximately 20% obstacles
        obstacle_count = len(obstacles)
        self.assertGreater(obstacle_count, 0)
        self.assertLess(obstacle_count, 25)  # At most 25 in 10x10
    
    def test_generate_with_density_level(self):
        """Test generation with density levels."""
        grid = Grid(10, 10)
        
        # Low density
        generator = ObstacleGenerator(grid, random_seed=42)
        obstacles = generator.generate_with_density_level('low', ensure_connectivity=False)
        
        # Low should have fewer obstacles
        self.assertLess(len(obstacles), 20)
    
    def test_ensure_connectivity(self):
        """Test that connectivity is ensured."""
        grid = Grid(10, 10)
        generator = ObstacleGenerator(grid, random_seed=42)
        
        obstacles = generator.generate_random(density=0.3, ensure_connectivity=True)
        
        # Path should exist from (0,0) to (9,9)
        # This is checked internally by ensure_connectivity


def run_tests():
    """Run all tests and return results."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestDijkstraCityNavigator))
    suite.addTests(loader.loadTestsFromTestCase(TestDijkstraGridNavigator))
    suite.addTests(loader.loadTestsFromTestCase(TestGrid))
    suite.addTests(loader.loadTestsFromTestCase(TestObstacleGenerator))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result


if __name__ == '__main__':
    run_tests()
