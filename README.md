<div align="center">

![ROS2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?style=for-the-badge&logo=gazebo&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Enabled-2496ED?style=for-the-badge&logo=docker&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-D22128?style=for-the-badge&logo=opensourceinitiative&logoColor=white)

</div>

# ROS 2 Development Workspace: Minibot Project

Este repositorio contiene un entorno de desarrollo para un robot autonomo con arquitectura ackermman utilizando **ROS2 Jazzy Jalisco** y **Gazebo Harmonic**.

El proyecto es un robot ackermman simulado y en fisico con equipo Lidar y una Camara, configurado para navegacion autonoma por medio de Nav2 yMapeo con Slam Toolbox.

## Estructura del proyecto

* **src/minibot**: paquete principal. Contiene la descripcion del robot en URDF/Xacro, mundos de simulacion, configuraciones de Nav2/SLAM y archivos de lanzamiento para simulador y en robot fisico.
* **src/ros_fundamentals_example**: paquete corto de fin educativo con ejemplos de Publishers y Suscribers escritos en C++ y Python.
* **Dockerfile** configuracion para utilizar un contenedor de Docker en todo el entorno de desarrollo.
* **Scripts de ayuda**:
    * `quick_build.sh`: Compilacion rapida con Colcon.
    * `robot_commands.sh`: Guia rapida de los comandos de teleoperacion.
    * `docker_robot_guide.sh`: Guia de uso del contenedor de docker.

## Instalacion y uso

Se puede utilizar de dos aneras, via Docker (Recomendado) o instalacion nativa.

### Utilizando Docker (Recomendado)

El repositorio incluye un `Dockerfile` optimizado basado en `osrf/ros:jazzy-desktop`.

1.  **Construir la imagen:**
    Asegúrate de estar en la raíz del repositorio:
    ```bash
    docker build -t ros_minibot_dev .
    ```

2.  **Ejecutar el contenedor:**
    Se recomienda usar un script o comando que habilite los gráficos (X11/Wayland) para ver Gazebo y RViz:
    ```bash
    # Ejemplo básico permitiendo acceso a Xhost
    xhost +local:root
    docker run -it --rm \
        --name minibot_container \
        --net=host \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        ros_minibot_dev
    ```

### Instalacion Nativa (Linux)

##### Requisitos
* Ubuntu 24.04 (Noble Numbat)
* ROS 2 Jazzy Jalisco
* Gazebo Harmonic

[https://docs.ros.org/en/jazzy/Installation.html](Instalacion ros2jazzy)

Tras la instalacion de ros2 jazzy, realizar lo siguiente:

1.  **Instalar dependencias:**
    ```bash
    sudo apt update
    sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-xacro ros-jazzy-robot-state-publisher
    ```

2.  **Compilar el workspace:**
    ```bash
    colcon build
    source install/setup.bash
    ```

## Comandos basicos
Una vez dentro del entorno (Docker o Nativo):

* **Compilar:** `colcon build`
* **Lanzar Simulación Completa:**
    ```bash
    ros2 launch minibot sim_world_robot.launch.py
    ```
    *(Ver documentación de `minibot` para más detalles)*

* **Ver Guía de Comandos:**
    ```bash
    ./robot_commands.sh
    ```
