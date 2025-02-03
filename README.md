# Automated Object Handling with UR5e 6-DoF Robotic Arm

*This document contains a demonstration of controlling the robotic arm in a simulated environment using a hybrid behaviour model.*

*Note: Credit for the Webots environment setup and kinematics transformation goes to Aston University.*

## Introduction

The demonstration involves tasking the robotic arm picking up 5 cubes on a flat table top surface, and then dropping them into a basket on the same table top. The goal is to implement such a behaviour framework, that enables reactive capabilities for the robotic arm with a degree of planning capacity, such that the cubes are chosen optimally, and if accidents happen, such as losing grip unexpectedly, logical actions would be taken in correspondence, e.g. alter the course of behaviour to recover the dropped cube.   

<div align="center">
  <img src="https://github.com/hongd13/Robotic-Arm/blob/master/pictures/intro.png?raw=true"/>
</div>

