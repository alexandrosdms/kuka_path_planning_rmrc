# 1. KUKA KR16 R1610-2 RMRC Path Planning Algorithm

## 1.1. Introduction
This a project assigned as part of a an undergraduate course in robotics in the University of Patras.
## 1.2. Backgroung
The aim of the project is for students to get familiar with the kinematics of robot manipulators and path planning algorithms. Each team chose an industrial robot arm in order to achive one of the following tasks:

1. Welding in a straight line
2. Placements of objects
3. Cutting process in a circular path
4. Rectangular path
5. Cubic interpolation
6. Linead and quadratic interpolation

In each process the required coordinate frames (robot base, targets, objects of the environment) are chosen by the student. Our team chose the straight line RMRC PP algorithm.

###   1.2.1. Welding in straight line
In this task it is required for the robot's end effector to move in a straight line in order to achive a welding process. The constraints of this process are:
    * The distance the tool must travel from point A to point B is 40cm.
    * The robot must cover the distance in 5cm/sec velocity.
### 1.2.2. Placements of objects
The robot is requred to pick 4 bottles from a container and place them one next to the other on a shelf. A point to point algorithm must be inccoporated to achive this task. The student is free to chose as many intermediate points as necessary.
### 1.2.3. Cutting process in a circular path
A circular cut is assumed to be needed on a block of textiles. The blade of the knife incoporated must always be in parallel with the tanget line or the path. The constraints of this path are:
* 20 cm diameter
* 10 cm/sec velocity
The RMRC algorithm must be incoporated to achieve this task.
### 1.2.4. Rectangular path
Glue must be placed in all sides of a rectangular frame. Constraints:
* The Taylor algorithm must be incoporated to achive the specified task. The task must be achieved with an accuracy of 0.1mm.
### 1.2.5. Cubic interpolation
Achieve a 3-point (non-collinearly)  interpolation mothin in space using a 6-degree of freedom robot arm. Velocity and accelaration must be continuous with no sudden changes. The time needed to pass between the first two points must be the same as the one needed to pass between the last two, and equal to double the time needed to achive a PtP algorithm.
### 1.2.6. Linead and quadratic interpolation

## The robot
For the straight line task a six degree of freedom industrial manipulator should be chosen. Our team chose the KUKA KR16 R1610-2 manipulator for its flexibility and easy modification for welding porpuses.

![alt text](https://www.quicktimeonline.com/assets/images/products/kr%2016%20r1610-2.jpg "KUKA KR16 R1610-2")

![image info](.pictures/kuka_axis.png "KUKA KR16 R1610-2 axis")

![image info](.pictures/kuka_workspace.png "KUKA KR16 R1610-2 Workspace")
  
## 1.3. RMRC Algorithm
## 1.4. Issues
## 1.5. Future updates