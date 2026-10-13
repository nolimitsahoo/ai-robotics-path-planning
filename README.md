# AI Robotics – Path Planning & Beacon Navigation

## Project Overview

This academic team project explored robot navigation using grid-based path planning and beacon-based waypoints. The system was designed to calculate routes through environments containing obstacles and visualize navigation behavior using ROS and RViz.

The project includes an A*-style path-planning implementation, beacon coordinate processing, and simulated robot movement along planned routes.

## Tools & Technologies

- **Python:** Path-planning algorithms and robot navigation logic
- **ROS 2 (rclpy):** Robot communication, publishers, subscribers, and node execution
- **ROS 1 (rospy):** Used in an earlier path-planning implementation
- **RViz:** Visualization of grid environments, waypoints, and robot positions
- **Grid-Based Search:** Path calculation across navigable cells and obstacles

## Project Features

### Grid-Based Path Planning
- Represents the environment using a two-dimensional grid.
- Distinguishes navigable cells from obstacles.
- Searches for a route between starting and destination coordinates.

### Beacon-Based Navigation
- Receives beacon coordinates through the ROS `/beacon_data` topic.
- Uses beacon locations as intermediate navigation waypoints.
- Calculates path segments from the starting location through beacon waypoints toward the destination.
- Includes beacon boundary and obstacle checks in the expanded implementation.

### Navigation Visualization
- Publishes RViz markers representing the grid, obstacles, start position, destination, beacons, and planned route.
- Updates a simulated robot position along the planned path.
- Publishes robot pose information through ROS.

## Project Files

| File | Description |
|---|---|
| `path_planner.py` | Expanded ROS 2 path planner with beacon handling, grid visualization, and simulated robot movement |
| `path_planner_beacons.py` | ROS 2 implementation of beacon-based waypoint navigation |
| `path_planning_a_star.py` | Earlier ROS 1 grid-based path-planning implementation |
| `setup.py` | ROS 2 Python package configuration |

## My Involvement

I participated in this academic team project, with involvement in the beacon-navigation portion of the work. The project provided experience working with Python, ROS-based navigation concepts, and grid-based path planning.

Individual code ownership has not been fully reconstructed from the original team development history.

## Technical Notes

The search implementation is named A* in the source code. Its current priority calculation uses accumulated path cost without an explicit goal-distance heuristic, making its behavior closer to uniform-cost search.

The code is preserved from the academic team project and has not been independently verified as runnable in its current repository structure.

## Team Acknowledgment

This project was completed collaboratively as part of an AI Robotics course. The original source repository was maintained by a teammate, and the implementation reflects the work of the project team.

## Project Context

**Project Type:** Academic Team Project

**Focus Areas:** Robot Navigation, Path Planning, Beacon Waypoints, ROS, Python
