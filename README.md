# Graph Search Algorithms

Python implementation of BFS, UCS, and A* algorithms for graph traversal and shortest path finding, with support for heuristic evaluation.

## Features

- **BFS (Breadth-First Search):** Explores states level by level, ignoring costs.  
- **UCS (Uniform Cost Search):** Finds the cheapest path considering edge costs.  
- **A\* (A-Star Search):** Combines actual cost and heuristic estimates for efficient pathfinding.  
- **Heuristic Checks:** Verify if heuristics are optimistic and consistent.

## Requirements

- Python 3.7+  
- Standard Python libraries: `heapq`, `collections`, `sys`

## Usage

Run the program from the command line:

```bash
python main.py --alg <algorithm> --ss <state_space_file> [--h <heuristic_file>] [--check-optimistic] [--check-consistent]
