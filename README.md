# Parallelized Synchronized Warehousing Robots (PSWR)

## Project Overview

The **Parallelized Synchronized Warehousing Robots (PSWR)** project is an innovative multi-agent robotic system designed for efficient warehouse automation. The system enables multiple autonomous robots to collaboratively retrieve and deliver pallets within a warehouse environment, ensuring smooth operation with synchronized parallel execution and collision avoidance.

Using **computer vision with ArUco markers** and **path planning algorithms**, the robots navigate dynamically changing warehouse layouts, optimizing routes and task assignments without relying on external frameworks like ROS. This project demonstrates an end-to-end solution encompassing localization, navigation, task coordination, and pallet handling.

---

## Key Features

- **Multi-agent path planning:** Utilizes the A* algorithm and Point & Shoot technique for real-time route planning and dynamic obstacle avoidance.
- **Robot localization:** Employs OpenCV’s ArUco marker detection for accurate positioning and orientation of robots within the warehouse.
- **Centralized command system:** A Python-based control center that assigns tasks, manages robot coordination, and monitors pallet status.
- **Collision avoidance:** Real-time adjustments to robot paths prevent collisions in a shared workspace.
- **Electromagnet gripper control:** Enables autonomous picking and placing of pallets with precision.
- **Parallel execution:** Simultaneous control and task execution of three robots with synchronized movement to maximize throughput.
- **No ROS dependency:** Custom socket communication and control algorithms provide flexibility and ease of integration.

---

## Technologies Used

- **Programming Language:** Python  
- **Computer Vision:** OpenCV (ArUco marker detection)  
- **Algorithms:** A* pathfinding, Point & Shoot navigation  
- **Communication:** TCP/IP socket programming for inter-robot and command center communication  
- **3D Modeling:** CAD files for robot and warehouse environment modeling  
- **Hardware:** Integration of camera systems and electromagnet grippers (simulation/prototype phase)

---

## Robot Images
![PHOTO-2025-04-13-19-37-51](https://github.com/user-attachments/assets/72bb3b7b-b380-4845-9bfa-75fa8f0600e9)

- **Front View**
![PHOTO-2025-04-13-19-37-51(3)](https://github.com/user-attachments/assets/01df8c3a-31ca-475c-91e9-796a4df1a684)
- **Top View**
![PHOTO-2025-04-13-19-41-49](https://github.com/user-attachments/assets/319c5ef1-859a-423d-9215-1f8b5eebad1f)
- **Side View**
![PHOTO-2025-04-13-19-37-51(2)](https://github.com/user-attachments/assets/a763f9b5-d34f-462e-8207-85d498cf2c99)

