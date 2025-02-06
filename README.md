# Automated Object Handling with UR5e 6-DoF Robotic Arm

*This document contains a demonstration of controlling the robotic arm in a simulated environment using a hybrid behaviour model.*

*Note: Credit for the Webots environment setup and kinematics transformation goes to Aston University.*

## Introduction

This demonstration showcases a behaviour model of a 6-DoF robotic arm, capable of detecting and locating objects, carrying out fine motor movements to align with the objects, grasping them and dropping the grasped objects into a predetermined location. 

The featured implementation is a behaviour-tree and state-machine hybrid. This model enables a highly adaptive behaviour with no fixed behaviour pattern while preserving a high-level error-handling capability. In other words, the model ensures that the robot carries out the tasks deliberately and predictably, while also being able to respond to unexpected scenarios or erroneous outcomes reactively in real-time, e.g. a pure state-machine model would result in behaviour such that the robot continues to the dropping location even if the object is dropped halfway. In contrast, the featured model enables the robot to react to and recover from the mishandling of objects or discover a more optimal movement pathing etc.

<div align="center">
  <img src="https://github.com/hongd13/Robotic-Arm/blob/master/pictures/intro.png?raw=true"/>
</div>

## Vision System

The robotic arm comes equipped with a camera mounted on the gripper, oriented to view in the same direction as the gripper's movement as shown below.

<div align="center">
  <img src="https://github.com/hongd13/Robotic-Arm/blob/master/pictures/cam_view_2.jpg?raw=true"/>
</div>

Colour thresholding and colour segmentation were performed for object identification. Given the distinct colour differences between the objects and table-top surfaces, these techniques provide robust and reliable object identification results. As shown below, the masked view after the colour thresholds were applied. 

<div align="center">
  <img src="https://github.com/hongd13/Robotic-Arm/blob/master/pictures/mask_view_2.jpg?raw=true"/>
</div>



## Robot in Action

<div align="center">
  <img src="https://github.com/hongd13/Robotic-Arm/blob/master/pictures/1%20cube%205.5x.gif?raw=true"/>
</div>

