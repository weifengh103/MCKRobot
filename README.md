# MCKROBOT  
## Let's Build a 3D-Printed 6-DOF Robot Arm from Scratch! 

<p align="center">
<img src="https://github.com/user-attachments/assets/515aa511-3345-4d26-aeb4-208ebae5a0b2" alt="Sample Image" style="width:80%; height:auto;">
</p>

<p align="center">
MCKRobot CAD Model
</p>

 

**MCKRobot** is a DIY project for building a 3D-printed, six degrees-of-freedom (6-DOF) robot arm. This repository includes two key components:  
1. **Kinematic Solver**: For robot motion calculations.  
2. **WPF GUI**: For robot control and visualization.  

---

## Kinematic Solver (Python, No ROS!)  

A lightweight kinematic solver tailored for the MCKRobot. It is specifically designed to solve Forward Kinematics (FK) and Inverse Kinematics (IK) for this 6-DOF robot arm and can be easily configured for other similar robots.  


<p align="center">
<img src="https://raw.githubusercontent.com/weifengh103/MCKRobot/refs/heads/master/Kinematic/Doc/MCKRobotGIF.gif" alt="Sample Image" style="width:60%; height:auto;">
</p>

<p align="center">
MCKRobot Kinematic simulation in Python
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
