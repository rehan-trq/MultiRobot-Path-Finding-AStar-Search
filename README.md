# Multi-Robot Path Planning with Dynamic Obstacles  

A Python-based implementation of multi-robot path planning using the A* algorithm, designed to handle both static and dynamic obstacles.  

## Overview  

This project simulates multiple robots navigating within a grid-world environment. Robots rely on the A* search algorithm to calculate efficient paths to their targets while adapting in real-time to the movement of dynamic agents and avoiding collisions with both static and moving obstacles.  

## Features  

- **Multiple Robots**: Supports independent start and destination points for each robot  
- **A* Pathfinding**: Classic A* search with Manhattan distance as the heuristic  
- **Dynamic Obstacle Handling**: Accounts for agents that move along predefined cyclic routes  
- **Collision Avoidance**: Ensures no two robots occupy the same cell simultaneously  
- **On-the-Fly Replanning**: Robots recompute paths when blocked by dynamic obstacles  
- **Simulation Metrics**: Records completion times and highlights robots that fail to progress  

## System Design  

### Main Components  

- **Robot Class**: Tracks each robot’s state, path, and progress  
- **A* Algorithm**: Pathfinding engine with temporal-spatial awareness  
- **Dynamic Agent Module**: Simulates moving obstacles with repeatable cyclic paths  
- **Simulation Manager**: Oversees robot movement, replanning, and collision checks  

### Core Algorithms  

1. **Temporal A***: Extends A* to include time as part of the search space  
2. **Manhattan Heuristic**: Provides a lightweight, grid-friendly distance estimate  
3. **Cyclic Agent Paths**: Agents travel forward and then backward repeatedly through their routes  

## Getting Started  

### Requirements  

- Python 3.7 or newer  
- Uses built-in libraries: `heapq`, `re`  

### Installation  

1. Clone the repository:  
```bash
git clone https://github.com/yourusername/multi-robot-pathfinding.git
cd multi-robot-pathfinding
