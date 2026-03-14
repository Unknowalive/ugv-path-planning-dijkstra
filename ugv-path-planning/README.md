# UGV Path Planning using Dijkstra Algorithm

A comprehensive Python implementation of path planning algorithms (Dijkstra and A*) for Unmanned Ground Vehicle (UGV) navigation in a grid environment with dynamic obstacle handling.

## 📋 Project Overview

This project implements:

1. **Part 1 - City Navigation**: Dijkstra's algorithm for finding shortest paths between cities in a weighted graph
2. **Part 2 - UGV Grid Navigation**: Pathfinding in a 70x70 grid with randomly generated obstacles
3. **Part 3 - Dynamic Obstacles**: Real-time obstacle detection and path replanning

## 🏗️ Project Structure

```
ugv-path-planning/
├── README.md                 # Project documentation
├── requirements.txt          # Python dependencies
├── main.py                   # Main entry point
├── config.py                 # Configuration settings
│
├── algorithms/
│   ├── dijkstra.py          # Dijkstra's algorithm implementation
│   └── astar.py             # A* algorithm implementation
│
├── environment/
│   ├── grid.py              # Grid environment class
│   └── obstacle_generator.py # Random obstacle generation
│
├── simulation/
│   ├── ugv.py               # UGV simulation class
│   └── dynamic_replanning.py # Dynamic obstacle handling
│
├── utils/
│   └── visualization.py     # Console visualization utilities
│
└── tests/
    └── test_dijkstra.py     # Unit tests
```

## 🔧 Installation

### Prerequisites
- Python 3.10 or higher

### Setup

1. Clone or download this repository
2. Navigate to the project directory:
   ```bash
   cd ugv-path-planning
   ```

3. No additional dependencies required (pure Python implementation)

## 🚀 Usage

### Running the Complete Demo

Run all three parts of the project:
```bash
python main.py
```

### Running Specific Modes

**City Navigation (Part 1):**
```bash
python main.py --mode city --from Delhi --to Mumbai
```

**UGV Navigation (Part 2):**
```bash
python main.py --mode ugv --algorithm dijkstra --density medium
```

**Dynamic Obstacles (Part 3):**
```bash
python main.py --mode dynamic --algorithm dijkstra
```

**Algorithm Comparison:**
```bash
python main.py --mode compare
```

**Run Tests:**
```bash
python main.py --mode test
```

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-m, --mode` | Mode: city, ugv, dynamic, compare, test, all | all |
| `-a, --algorithm` | Algorithm: dijkstra, astar | dijkstra |
| `-d, --density` | Obstacle density: low, medium, high | medium |
| `-g, --grid-size` | Grid dimensions (width height) | 20 20 |
| `-f, --from` | Starting city | Delhi |
| `-t, --to` | Goal city | Mumbai |
| `--no-visualize` | Disable visualization | False |

## 📖 Algorithm Explanation

### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path between nodes in a weighted graph. It uses a priority queue to explore nodes in order of their distance from the source.

**Key Features:**
- Guarantees shortest path
- Uses greedy approach with priority queue
- Time Complexity: O((V + E) log V)
- Space Complexity: O(V)

### A* Algorithm

A* is an informed search algorithm that uses heuristics to guide its search, making it more efficient than Dijkstra's for pathfinding in grids.

**Key Features:**
- Uses heuristic function h(n) to estimate cost to goal
- f(n) = g(n) + h(n) where g(n) is cost from start
- More efficient than Dijkstra in open grids
- Optimal when heuristic is admissible

## 📊 Example Output

### City Navigation

```
============================================================
PART 1: CITY NAVIGATION USING DIJKSTRA'S ALGORITHM
============================================================

Finding shortest path from Delhi to Mumbai...

Shortest Path: Delhi -> Jaipur -> Ahmedabad -> Mumbai
Total Distance: 1942 km

Journey Details:
  Delhi -> Jaipur: 280 km
  Jaipur -> Ahmedabad: 295 km
  Ahmedabad -> Mumbai: 1367 km
```

### UGV Grid Navigation

```
============================================================
PART 2: UGV NAVIGATION IN GRID ENVIRONMENT
============================================================

Configuration:
  Grid Size: 20x20
  Obstacle Density: medium (25%)
  Algorithm: dijkstra

Path found!
  Path Length: 38 steps
  Total Cost: 37.0
  Explored Nodes: 156

Grid Visualization:
════════════════════════════╗
│                            ║
│ S●●●●●●●●●●●●●●●●●●●●●●●G  ║
│ ███ ██████████████████████ ░║
│     ██████████████████████░░║
│ ████████ █████████████████░║
│           ████████████████░░║
│ █████████████████████████░░░║
│ █████████████████████████░░░║
│ ████████████████          ░║
│ █████████████████████████░░░║
│                    ████████░║
│ █████████████████████████░░░║
│           █████████████████░║
│ ██████████████████████████░░║
│ ██████████████████████████░░║
│ ████████████               ░║
│ ██████████████████████████░░║
│ ██████████████████████████░░║
│                  ██████████░║
│ ██████████████████████████░░░║
│                           ░║
════════════════════════════╝

Legend:  S Start |  G Goal |  ● Path |  · Explored |  █ Obstacle |    Free
```

### Dynamic Replanning

```
============================================================
PART 3: DYNAMIC OBSTACLES AND REPLANNING
============================================================

Starting dynamic replanning simulation...
Start: (0, 0), Goal: (19, 19)
Algorithm: dijkstra
Obstacle probability: 0.15

Step 0: Position (0, 0), At goal: False
  -> Path replanned! (Total replans: 1)
Step 100: Position (45, 17), At goal: False
Step 200: Position (19, 19), At goal: True

==================================================
Simulation Complete!
Goal reached: True
Total steps: 234
Replan count: 3
Dynamic obstacles added: 5
Obstacles blocking path: 2
```

## 🎯 Features

### Part 1: City Navigation
- ✅ Weighted graph representation of cities
- ✅ Dijkstra's algorithm implementation
- ✅ Support for custom graphs
- ✅ Distance calculation between any two cities

### Part 2: UGV Navigation
- ✅ 70x70 grid environment (configurable)
- ✅ Random obstacle generation with density levels:
  - Low: 15% obstacle density
  - Medium: 25% obstacle density
  - High: 40% obstacle density
- ✅ Path visualization in console
- ✅ Both Dijkstra and A* algorithms
- ✅ Explored nodes visualization

### Part 3: Dynamic Obstacles
- ✅ Obstacles appear during navigation
- ✅ Automatic path replanning when blocked
- ✅ Statistics tracking (replans, blocked paths)
- ✅ Demonstration of adaptive behavior

## 🧪 Testing

Run unit tests:
```bash
python -m pytest tests/test_dijkstra.py
```

Or using the main module:
```bash
python main.py --mode test
```

Test coverage includes:
- Dijkstra city navigation
- Dijkstra grid navigation
- Grid operations
- Obstacle generation
- Path validation

## 📝 Configuration

All settings can be modified in [`config.py`](config.py):

```python
# Grid settings
GRID_WIDTH = 70
GRID_HEIGHT = 70

# Obstacle density
OBSTACLE_DENSITY = {
    'low': 0.15,
    'medium': 0.25,
    'high': 0.40
}

# Algorithm settings
DEFAULT_ALGORITHM = 'dijkstra'
MOVE_COST = 1
```

## 🔬 Technical Details

### Grid Representation
- 0 represents free cells
- 1 represents obstacles
- 4-directional movement (up, down, left, right)

### Pathfinding
- Uses priority queue (heapq) for efficient node selection
- Tracks explored nodes for visualization
- Calculates total path cost

### Dynamic Replanning
- Monitors current path for obstacles
- Triggers replanning when path is blocked
- Maintains navigation state across replans

## 📚 Academic Context

This project is designed for educational purposes to demonstrate:

1. **Graph Algorithms**: Understanding Dijkstra's and A* implementations
2. **Path Planning**: Real-world application of search algorithms
3. **Dynamic Systems**: Handling changing environments
4. **Python Best Practices**: Modular design, docstrings, testing

## 🤝 Contributing

Feel free to enhance this project by:
- Adding new algorithms (Bidirectional search, D*)
- Implementing visualization (matplotlib, pygame)
- Adding path smoothing
- Implementing sensor models

## 📄 License

This project is for educational purposes.

## 👥 Authors

- UGV Path Planning Team

## 🔗 References

- Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs"
- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
