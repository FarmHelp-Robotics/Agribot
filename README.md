# Agribot: Vision-Guided Tomato Harvesting Robot  

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)  
[![Jetson Nano](https://img.shields.io/badge/NVIDIA-Jetson%20Nano-green)](https://developer.nvidia.com/embedded/jetson-nano)  
[![YOLOv8](https://img.shields.io/badge/YOLO-v8-red)](https://github.com/ultralytics/ultralytics)  

<img width="338" height="605" alt="Screenshot from 2025-09-01 11-43-52" src="https://github.com/user-attachments/assets/7978684d-7193-474b-a2de-e062c64c2acf" />



> Affordable agricultural robot designed to automate tomato harvesting in small & medium-scale farms in India.  

---

## Overview  
Agribot is a **vision-guided autonomous harvesting robot**.  
It integrates a tracked mobile base, a 6-DOF robotic arm, and a flexible gripper with an ML-based perception pipeline to **detect, localize, and gently pluck ripe tomatoes**.  

---

## Features  
- **Tracked base** (Yahboom Transbot) for rough farm terrain.  
- **6-DOF robotic arm** (MyCobot 280 Arduino) with MoveIt planning.  
- **Stereo vision** (Intel RealSense D405) for 3D fruit localization.  
- **Tomato detection** with YOLOv8-nano trained on India-specific dataset.  
- **Custom flexible gripper** with torque-optimized design.  
- Runs fully **onboard Jetson Nano (9–11W)** with real-time inference.  

---

## System Workflow  


1. Detect tomatoes with YOLOv8-nano (RGB-D input).  
2. Convert image coords → 3D world coords.  
3. TRAC-IK solver computes valid joint angles.  
4. OMPL motion planner (RRT-Connect) generates safe path.  
5. Arduino executes trajectory on the robotic arm.  
6. Gripper plucks tomato & drops into collection basket.  

**Avg harvest cycle**: 15–17 sec / tomato  

---

## ⚙Hardware  

- **Base**: Yahboom Transbot (tracked)  
- **Arm**: MyCobot 280 Arduino, 6-DOF  
- **Gripper**: Flexible silicon fingers + high-torque servo  
- **Camera**: Intel RealSense D405 RGB-D  
- **Compute**: NVIDIA Jetson Nano Dev Kit  
- **Power**: 12V Li-ion battery  

---

## Software  

- **OS**: Ubuntu 20.04 LTS  
- **Middleware**: ROS Noetic + MoveIt  
- **Perception**: YOLOv8-nano (PyTorch, Ultralytics), OpenCV, RealSense SDK  
- **Planning**: TRAC-IK solver + OMPL (RRT-Connect)  
- **Control**: Arduino IDE + ROS serial communication  
- **Dataset Annotation**: Label Studio  

---

## Results  

- **Detection F1-Score**: 0.60 (ripe tomato detection)  
- **Harvest Time**: 15–17 sec per cycle  
- **Power Usage**: 9–11 W (Jetson Nano)  
- **Inference Latency**: ~0.13 ms  

## Installation
```bash
# from root of the workspace
mkdir -p agribot_ws/src && cd agribot_ws/src # Skip this if you already have a workspace

git clone https://github.com/FarmHelp-Robotics/agribot_src
rosdep install --from-paths src --ignore-src -r -y

# enjoy! :D
```
## 

## Test Video of Agribot 
Agribot plucking tomato in test bed

https://github.com/user-attachments/assets/05b39af2-fc81-404d-a115-3bdc4d79b47c


https://github.com/user-attachments/assets/148897a3-5d52-45b9-a57e-114927ce68bf


