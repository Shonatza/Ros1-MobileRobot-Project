
# Short Project on Mobile Robots

This repository contains the code and implementation of a robotics project developed for the robotics course at FIB. The project focuses on designing and integrating multiple control modes for a TurtleBot3 using ROS1, offering an interactive and modular framework.

## Key Features

### Multi-Mode Functionality
The robot can switch between four modes effortlessly:
1. **Manual Control:** Control via keyboard inputs (WASDX keys).
2. **Reflective Object Tracking:** Follow highly reflective objects using LIDAR data.
3. **General Object Tracking:** Adaptable tracking for any nearby object with custom intensity detection.
4. **SLAM Mode:** Real-time mapping and localization using existing ROS1 packages with RViz and Gmapping.

### User-Friendly Interface
Designed to minimize terminal commands and user input by handling mode switching and process management through a single main program.

### Technical Insights
Implemented in C++, leveraging ROS1 metapackages for scalability and ease of integration. The project tackles challenges like varying robot sensor behaviors and real-world hardware constraints.

The **README file** serves as a critical component of the project, offering clear instructions on setup, configuration, and usage.

---
