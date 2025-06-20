# Segurito

![Project Logo](path/to/logo.png)

## Table of Contents

1. [Introduction](#introduction)  
2. [3D Model](#3d-model)  
3. [Features](#features)  
4. [Components & Costs](#components--costs)  
5. [Architecture Diagrams](#architecture-diagrams)  
   - [Hardware Diagram](#hardware-diagram)  
   - [Software Diagram](#software-diagram)  
6. [Installation](#installation)  
   - [Requirements](#requirements)  
   - [Setup Guide](#setup-guide)  
7. [Usage](#usage)  
   - [LiDAR Scanning & Mapping](#lidar-scanning--mapping)  
   - [Motion Detection](#motion-detection)  
   - [Object & Face Recognition](#object--face-recognition)  
   - [Remote Monitoring](#remote-monitoring)  
8. [Algorithms](#algorithms)  
9. [Simulation](#simulation)  
10. [Documentation & Examples](#documentation--examples)  
11. [Contributors](#contributors)  
12. [License](#license)  

---

## Introduction

Segurito is an autonomous security robot designed to patrol and monitor spaces as a dedicated security personnel would. Equipped with advanced sensors and cloud connectivity, Segurito:

- Maps its environment using LiDAR and SLAM algorithms  
- Detects obstacles and moving objects for safe navigation  
- Identifies humans and animals via camera-based recognition  
- Authenticates authorized personnel using facial recognition  
- Sounds alarms and sends remote notifications upon intruder detection  
- Listens for voice commands through an onboard microphone  

Segurito combines robust hardware with intelligent software modules to deliver a comprehensive security solution.

---

## 3D Model

The complete robot chassis and component mounts have been designed in 3D CAD. You can find the model files here:

- `models/segurito_base.stl`  
- `models/lidar_mount.stl`  
- `models/camera_holder.stl`  

<details>  
  <summary>Preview GIF of 3D Assembly</summary>

  ![3D Assembly Preview](path/to/3d_model_preview.gif)  
</details>

---

## Features

- **Autonomous Mapping**: Builds a real‑time map of the environment using LiDAR and SLAM.  
- **Collision Avoidance**: Detects and avoids obstacles.  
- **Motion Detection**: PIR sensor for enhanced detection accuracy.  
- **Human & Animal Recognition**: Classifies figures captured by the camera.  
- **Facial Authentication**: Recognizes authorized faces to grant access.  
- **Alarm System**: Sounds buzzer upon intruder detection.  
- **Voice Commands**: Microphone for receiving commands.  
- **Cloud Connectivity**: Sends alerts and live video feed to a remote dashboard.  

---

## Components & Costs

| Component                   | Quantity | Approx. Cost (USD) | Vendor Link               |
|-----------------------------|:--------:|-------------------:|---------------------------|
| LiDAR Sensor (YDLIDAR X4)   |    1     |            350.00  | [Link](https://...)       |
| Raspberry Pi 4B (4 GB RAM)  |    1     |             55.00  | [Link](https://...)       |
| PIR Motion Sensor           |    1     |              5.00  | [Link](https://...)       |
| Pi Camera HQ                |    1     |             50.00  | [Link](https://...)       |
| Buzzer & Microphone Module  |    1     |             10.00  | [Link](https://...)       |
| Battery Pack (12 V, 5000 mAh)|   1     |             45.00  | [Link](https://...)       |
| Motor Controllers           |    2     |             40.00  | [Link](https://...)       |
| 3D‑Printed Chassis          |    1     |             70.00  | —                         |
| **Total**                   |          |           **625.00**|                           |

---

## Architecture Diagrams

### Hardware Diagram

![Hardware Diagram](path/to/hardware_diagram.png)

### Software Diagram

![Software Diagram](path/to/software_diagram.png)

---

## Installation

### Requirements

**Hardware**  
- Raspberry Pi 4B (4 GB RAM) with Raspbian OS  
- YDLIDAR X4 (or similar compatible LiDAR)  
- Pi Camera HQ  
- PIR motion sensor  
- Buzzer & microphone module  
- Motor drivers & DC motors  
- 12 V battery pack  

**Software**  
- Python 3.9+  
- OpenCV 4.x  
- ROS Noetic  
- NumPy, SciPy, Scikit‑learn  
- SLAM library (e.g., `rtabmap_ros`)  
- TensorFlow or PyTorch (for recognition models)  

### Setup Guide

1. **Flash OS**  
   ```bash
   # Write Raspbian image to SD card
   sudo dd if=raspbian.img of=/dev/sdX bs=4M status=progress
