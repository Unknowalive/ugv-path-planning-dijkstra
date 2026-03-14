"""
Configuration settings for UGV Path Planning project.

This module contains all configuration parameters used throughout
the project, including grid settings, obstacle densities, and
algorithm parameters.

Author: UGV Path Planning Team
Version: 1.0.0
"""

# Grid Environment Configuration
GRID_WIDTH = 70
GRID_HEIGHT = 70

# Obstacle Density Levels (probability of cell being an obstacle)
OBSTACLE_DENSITY = {
    'low': 0.15,
    'medium': 0.25,
    'high': 0.40
}

# Default obstacle density
DEFAULT_OBSTACLE_DENSITY = 'medium'

# UGV Configuration
UGV_START_POSITION = (0, 0)
UGV_GOAL_POSITION = (69, 69)

# Algorithm Configuration
DEFAULT_ALGORITHM = 'dijkstra'  # Options: 'dijkstra', 'astar'

# Pathfinding Costs
MOVE_COST = 1          # Cost for moving to adjacent cell (4-directional)
DIAGONAL_MOVE_COST = 1.414  # Cost for diagonal movement (if enabled)

# Visualization Configuration
VISUALIZATION_DELAY = 0.01  # Delay between visualization steps (seconds)
CONSOLE_COLORS = True       # Enable colored console output

# City Navigation Graph Configuration
DEFAULT_CITY_GRAPH = {
    'Delhi': {'Mumbai': 1157, 'Kolkata': 1471, 'Chennai': 2201, 'Bangalore': 2150},
    'Mumbai': {'Delhi': 1157, 'Bangalore': 983, 'Chennai': 1334, 'Kolkata': 1677},
    'Bangalore': {'Mumbai': 983, 'Delhi': 2150, 'Chennai': 350, 'Kolkata': 1567},
    'Chennai': {'Bangalore': 350, 'Mumbai': 1334, 'Delhi': 2201, 'Kolkata': 1683},
    'Kolkata': {'Delhi': 1471, 'Mumbai': 1677, 'Bangalore': 1567, 'Chennai': 1683},
    'Hyderabad': {'Bangalore': 570, 'Mumbai': 660, 'Chennai': 630, 'Delhi': 1250},
    'Ahmedabad': {'Mumbai': 530, 'Delhi': 915, 'Jaipur': 295, 'Surat': 270},
    'Jaipur': {'Delhi': 280, 'Ahmedabad': 295, 'Kolkata': 1450, 'Mumbai': 1170},
    'Surat': {'Mumbai': 260, 'Ahmedabad': 270, 'Bangalore': 1160, 'Hyderabad': 760},
    'Pune': {'Mumbai': 150, 'Bangalore': 840, 'Hyderabad': 560, 'Delhi': 1400}
}

# Dynamic Obstacle Simulation
DYNAMIC_OBSTACLE_CHANCE = 0.05  # Probability of new obstacle appearing per step
DYNAMIC_OBSTACLE_COUNT = 3     # Number of obstacles that can appear

# Testing Configuration
TEST_GRID_SIZE = 10
TEST_OBSTACLE_DENSITY = 0.2

# Output Configuration
SHOW_EXPLORED_NODES = True
SHOW_PATH = True
SHOW_OBSTACLES = True
