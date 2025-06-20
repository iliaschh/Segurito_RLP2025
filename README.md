# Segurito 

![Logo del Proyecto](path/to/logo.png)

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Modelo 3D](#modelo-3d)
3. [Características](#características)
4. [Componentes y Costos](#componentes-y-costos)
5. [Diagramas de Arquitectura](#diagramas-de-arquitectura)
   - [Diagrama de Hardware](#diagrama-de-hardware)
   - [Diagrama de Software](#diagrama-de-software)
6. [Instalación](#instalación)
   - [Requisitos](#requisitos)
   - [Guía de Instalación](#guía-de-instalación)
7. [Uso](#uso)
   - [Escaneo y Mapeo LiDAR](#escaneo-y-mapeo-lidar)
   - [Detección de Movimiento](#detección-de-movimiento)
   - [Reconocimiento de Objetos y Rostros](#reconocimiento-de-objetos-y-rostros)
   - [Monitoreo Remoto](#monitoreo-remoto)
8. [Algoritmos](#algoritmos)
9. [Simulación](#simulación)
10. [Documentación y Ejemplos](#documentación-y-ejemplos)
11. [Contribuidores](#contribuidores)
12. [Licencia](#licencia)

---

## Introducción

Segurito es un robot de seguridad autónomo diseñado para patrullar y supervisar espacios como lo haría un vigilante dedicado. Equipado con sensores avanzados y conectividad en la nube, Segurito:

- Mapea su entorno usando algoritmos LiDAR y SLAM
- Detecta obstáculos y objetos en movimiento para una navegación segura
- Identifica humanos y animales mediante reconocimiento por cámara
- Autentica al personal autorizado usando reconocimiento facial
- Activa alarmas y envía notificaciones remotas al detectar intrusos
- Atiende comandos de voz a través de un micrófono integrado

Segurito combina hardware robusto con módulos de software inteligentes para ofrecer una solución de seguridad integral.

---

## Modelo 3D

El chasis completo del robot y los soportes de los componentes han sido diseñados en CAD 3D. Puedes encontrar los archivos del modelo aquí:

- `models/segurito_base.stl`
- `models/lidar_mount.stl`
- `models/camera_holder.stl`

<details>
  <summary>GIF de Vista Previa del Ensamblaje 3D</summary>

  ![Vista Previa 3D](path/to/3d_model_preview.gif)
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
