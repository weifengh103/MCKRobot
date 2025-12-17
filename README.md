# MCKROBOT  
## Let's Build a 3D-Printed 6-DOF Robot Arm from Scratch! 

<p align="center">
<img src="https://github.com/weifengh103/MCKRobot/blob/master/Readme%20access/CAD%20+%20Assembly.png?raw=true" alt="Sample Image" style="width:180%; height:auto;">
</p>

<p align="center">
MCKRobot CAD Model and Assembly
</p>

 

**MCKRobot** is a DIY project for building a 3D-printed, six degrees-of-freedom (6-DOF) robot arm. This repository includes two key components:  
1. **Kinematic Solver**: For robot motion calculations.
2. **Robot Build and Control**: For building the actual robot.  
3. **WPF GUI**: For robot control and visualization.  

---

## Kinematic Solver (Python, No ROS!)  

A lightweight kinematic solver tailored for the MCKRobot. It is specifically designed to solve Forward Kinematics (FK) and Inverse Kinematics (IK) for this 6-DOF robot arm and can be easily configured for other similar robots.  


<p align="center">
<img src="https://raw.githubusercontent.com/weifengh103/MCKRobot/refs/heads/master/Kinematic/Doc/MCKRobotGIF.gif" alt="Sample Image" style="width:60%; height:auto;">
</p>

<p align="center">
Kinematic simulation for TCP RX, RY, RZ, X, Y, Z movement
</p>

### **Forward Kinematics (FK)**  
The FK solver calculates the robot's Tool Center Point (TCP) or flange pose based on the provided joint angles. It uses the **Denavit-Hartenberg (DH) parameterization method**, which simplifies calculations by defining the coordinates and dimensions of each joint. Once these parameters are set, the robot's pose can be efficiently determined using the DH matrix.  

### **Inverse Kinematics (IK)**  
The IK solver computes the joint angles required to achieve a given TCP pose (T_base-TCP). This enables precise motor control for the robot's motion.  

**Key components of the IK solver:**  
1. **Position [X, Y, Z]:**  
   Solved using joints J1, J2, and J3 through a geometric approach.  
2. **Orientation [RX, RY, RZ]:**  
   Solved using joints J4, J5, and J6, which are treated as a spherical joint with three rotational degrees of freedom. Orientation is computed using the **Euler angle approach** in XYZ order.  

---

## Robot Build and Control

The robot was designed using Autodesk Fusion 360 and built with a combination of 3D-printed and off-the-shelf components.

Key items:

- 3D-printed housing and belt gears

- Stepper motors and drivers for joint movement

- ESP32 controller (may switch to Raspberry Pi later)

<p align="center"> <img src="https://github.com/weifengh103/MCKRobot/blob/master/Readme%20access/MCK%20Robot%20GIF%202.gif?raw=true" alt="MCK Robot Demonstration" style="width:60%; height:auto;"> </p> 

<p align="center">
Individual joint testing
</p>

---

## WPF GUI (Work In Progress)  


The WPF GUI provides an intuitive interface for controlling and programming the robot. Features include:  
- Controls and programing for robot motion.  
- Real-time visualization of the robot's movements.

<p align="center">
<img src="https://github.com/user-attachments/assets/4def2f8d-0ac4-4394-9bee-05e52ba405ff" alt="Sample Image" style="width:80%; height:auto;">
</p>

<p align="center">
MCKRobot Visulization in WPF Helix
</p>



---
