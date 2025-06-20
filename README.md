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

## Características

- **Mapeo Autónomo**: Genera un mapa en tiempo real del entorno usando LiDAR y SLAM.
- **Evitación de Colisiones**: Detecta y evita obstáculos.
- **Detección de Movimiento**: Sensor PIR para mayor precisión.
- **Reconocimiento de Humanos y Animales**: Clasifica las figuras capturadas por la cámara.
- **Autenticación Facial**: Reconoce rostros autorizados para otorgar acceso.
- **Sistema de Alarma**: Activa un zumbador al detectar intrusos.
- **Comandos de Voz**: Micrófono para recibir instrucciones.
- **Conectividad en la Nube**: Envía alertas y transmisión de video en vivo a un panel remoto.


---

## Componentes y Costos

| Componente                  | Cantidad | Costo Aprox. (USD) | Enlace al Proveedor     |
|-----------------------------|:--------:|-------------------:|-------------------------|
| Sensor LiDAR (YDLIDAR X4)   |    1     |            350.00  | [Enlace](https://...)   |
| Raspberry Pi 4B (4 GB RAM)  |    1     |             55.00  | [Enlace](https://...)   |
| Sensor de Movimiento PIR    |    1     |              5.00  | [Enlace](https://...)   |
| Pi Camera HQ                |    1     |             50.00  | [Enlace](https://...)   |
| Módulo Zumbador y Micrófono |    1     |             10.00  | [Enlace](https://...)   |
| Batería (12 V, 5000 mAh)    |    1     |             45.00  | [Enlace](https://...)   |
| Controladores de Motores    |    2     |             40.00  | [Enlace](https://...)   |
| Chasis Impreso en 3D        |    1     |             70.00  | —                       |
| **Total**                   |          |           **625.00**|                         |

---

## Diagramas de Arquitectura

### Diagrama de Hardware

![Diagrama de Hardware](path/to/hardware_diagram.png)

### Diagrama de Software

![Diagrama de Software](path/to/software_diagram.png)

---

## Instalación

### Requisitos

**Software**

- Python 3.9+
- OpenCV 4.x
- ROS Noetic
- NumPy, SciPy, Scikit‑learn
- Librería SLAM (p. ej., `rtabmap_ros`)
- TensorFlow o PyTorch (para modelos de reconocimiento)

### Guía de Instalación

1. **Flashear el Sistema Operativo**
   ```bash
   sudo dd if=raspbian.img of=/dev/sdX bs=4M status=progress
   ```
2. **Actualizar el Sistema**
   ```bash
   sudo apt-get update && sudo apt-get upgrade -y
   ```
3. **Instalar ROS Noetic**
   ```bash
   sudo apt install ros-noetic-desktop-full
   source /opt/ros/noetic/setup.bash
   ```
4. **Instalar el Driver del LiDAR**
   ```bash
   sudo apt install ros-noetic-ydlidar
   ```
5. **Clonar el Repositorio**
   ```bash
   git clone https://github.com/yourusername/segurito.git
   cd segurito
   ```
6. **Instalar Dependencias de Python**
   ```bash
   pip install -r requirements.txt
   ```
7. **Compilar y Ejecutar**
   ```bash
   catkin_make
   source devel/setup.bash
   roslaunch segurito main.launch
   ```
---

## Uso

### Escaneo y Mapeo LiDAR
   ```bash
      rosrun segurito lidar_mapping.py \
     --scan_topic /scan \
     --output_map maps/segurito_map.pgm
   ```
Genera una barrida de 360°, recoge nubes de puntos y guarda un mapa de ocupación.

### Detección de Movimiento
   ```bash
      python3 src/motion_detector.py
   ```
Monitorea el sensor PIR conectado al GPIO 17.

### Reconocimiento de Objetos y Rostros
   ```bash
      rosrun segurito recognition_node.py \
     --model models/face_recognition.pkl
   ```
Monitorea el sensor PIR conectado al GPIO 17.

### Monitoreo Remoto
Accede a video en vivo y alertas en: [Enlace](https://...)

