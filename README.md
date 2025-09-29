# Autonomous Robot Workspace

## Vision general
Este repositorio es un espacio de trabajo de ROS&nbsp;2 orientado al desarrollo de un robot autonomo basado en la plataforma **Rosmaster X3** con cinematica omnidireccional mediante ruedas Mecanum. La pieza central del proyecto es el paquete `autonomous_robot_description`, que contiene la descripcion URDF/Xacro del robot, los recursos 3D y la integracion con Gazebo. El paquete `autonomous_robot` actua como metapaquete y punto de entrada para futuras funcionalidades de control y navegacion.

> Nota: El workspace incluye el paquete `ros2_fundamentals_examples`, pensado unicamente como material educativo para aprender ROS. No forma parte del alcance del proyecto final y puede ignorarse salvo que se requieran ejemplos introductorios.

## Estructura del repositorio
- `autonomous_robot/`: Metapaquete que depende de la descripcion del robot y servira como agregador de nodos de alto nivel en el futuro.
- `autonomous_robot_description/`: Contiene los archivos Xacro/URDF, mallas STL, futuros lanzadores y configuraciones para RViz y Gazebo.
  - `urdf/robots/rosmaster_x3.urdf.xacro`: Ensambla la base, las ruedas, la camara RGB-D, el lidar y la IMU.
  - `urdf/mech/`: Macros para la base y las ruedas Mecanum.
  - `urdf/sensors/`: Macros parametrizables para la camara Intel RealSense D435, el lidar RPLidar S2 y una IMU generica.
  - `meshes/`: Modelos 3D en formato STL para la visualizacion en RViz/Gazebo.
- `ros2_fundamentals_examples/`: Ejercicios de publicacion/suscripcion en Python y C++ (no usados en el proyecto final).

## Requisitos previos
- Ubuntu 22.04 LTS (o compatible) con **ROS 2 Humble Hawksbill** instalado.
- `colcon` y extensiones comunes para compilar workspaces (`python3-colcon-common-extensions`).
- Paquetes ROS 2 adicionales relacionados con URDF, Xacro, RViz y Gazebo:
  ```bash
  export ROS_DISTRO=humble   # ajusta si trabajas con otra distribucion
  sudo apt update
  sudo apt install \
    ros-$ROS_DISTRO-urdf-tutorial \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-rviz2
  ```
- Gazebo Classic (incluido en `gazebo-ros-pkgs`) para simulacion.
- Asegurate de tener los drivers propietarios necesarios para visualizar mallas STL si usas tarjetas graficas dedicadas.

## Instalacion y compilacion
1. Crea o posiciona tu workspace:
   ```bash
   mkdir -p ~/ros_dev/src
   cd ~/ros_dev
   ```
2. Clona o copia este repositorio dentro de `src/` y actualiza las dependencias del sistema segun el apartado anterior.
3. Compila el workspace:
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   colcon build --symlink-install
   ```
4. Activa el overlay resultante para cada nueva terminal:
   ```bash
   source install/setup.bash
   ```

## Uso principal
### Generar el URDF expandido (a partir de Xacro)
```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix autonomous_robot_description)/share/autonomous_robot_description/urdf/robots/rosmaster_x3.urdf.xacro \
  -o /tmp/rosmaster_x3.urdf
```

### Visualizar en RViz2
```bash
ros2 launch urdf_tutorial display.launch.py model:=/tmp/rosmaster_x3.urdf
```
Esto abrira RViz2 con `robot_state_publisher` y `joint_state_publisher_gui`. Ajusta las articulaciones de las ruedas desde la interfaz para comprobar el estado cinematico.

### Cargar el robot en Gazebo Classic
```bash
# En una terminal
ros2 launch gazebo_ros gazebo.launch.py

# En otra terminal
ros2 run gazebo_ros spawn_entity.py \
  -file /tmp/rosmaster_x3.urdf \
  -entity rosmaster_x3
```
Los sensores publicados desde Gazebo utilizaran los topicos definidos en los Xacro (`scan`, `imu/data`, `cam_1`).

### Personalizar prefijos o instancias
El archivo principal `rosmaster_x3.urdf.xacro` acepta argumentos como `prefix` para instanciar multiples robots y `use_gazebo` (reservado para futuras extensiones). Ejemplo:
```bash
ros2 run xacro xacro .../rosmaster_x3.urdf.xacro prefix:=robot1_ -o /tmp/robot1.urdf
```

## Detalles tecnicos del robot URDF
**Base y estructura**

| Parametro | Valor |
|-----------|-------|
| Longitud base (`base_length`) | 0.300 m |
| Anchura util (`base_size_y`) | 0.1386 m |
| Altura de la caja de colision | 0.19725 m |
| Altura total del chasis | 0.26225 m |
| Masa base | 4.6 kg |
| Material visual | Verde (RGBA 0, 0.7, 0, 1) |

**Ruedas Mecanum**

| Aspecto | Valor |
|---------|-------|
| Radio | 0.0325 m |
| Ancho | 0.0304 m |
| Masa (cada rueda) | 9.1 kg |
| Separacion lateral (centro a centro) | 0.169 m |
| Offset longitudinal | +/-0.08 m (frente/trasera) |
| Offset lateral | +/-0.0745 m |
| Articulaciones | `continuous`, eje Y |
| Friccion Gazebo | `mu=1.0`, `mu2=0.0` |

**Sensor RGB-D (Intel RealSense D435)**

- Montaje: `base_link` con offset `(0.105, 0, 0.05)` y pitch de -0.5 rad.
- Masa e inercias realistas (0.072 kg) y malla STL incluida.
- Field of view 1.5184 rad, resolucion 424x240.
- Publica en Gazebo a 2 Hz sobre `/cam_1` con frame optico `cam_1_depth_optical_frame`.

**Lidar (RPLidar S2)**

- Ubicacion: `(0, 0, 0.0825)` sobre `base_link`.
- Rango 0.20 m - 30 m, 720 muestras, cobertura 360 deg.
- Publica en `/scan` a 10 Hz. Malla STL escalada para visualizacion.

**IMU**

- Caja de 38.9 mm x 37.9 mm x 13.4 mm anclada al centro del `base_link`.
- Masa 0.031 kg, publica en `/imu/data` a 15 Hz.

**Jerarquia de frames relevante**
- `base_footprint` --> `base_link` --> ruedas (`*_wheel_link`).
- Sensores: `cam_1_link`, `laser_frame`, `imu_link` con sus respectivos frames opticos/hijos.
- Todos los sensores utilizan la convencion REP-103 (z hacia arriba en base, frames opticos con z adelante/y abajo).

## Paquetes auxiliares
- `ros2_fundamentals_examples`: ejercicios minimos de publicacion/suscripcion en Python y C++. Utiles como referencia rapida, pero no intervienen en la arquitectura del robot autonomo.

## Proximos pasos sugeridos
1. Anadir lanzadores en `autonomous_robot_description/launch/` para automatizar RViz y Gazebo.
2. Incluir nodos de control en `autonomous_robot/` (p. ej. controlador de cinematica Mecanum, fusion de sensores).
3. Incorporar perfiles de `ros2_control` y `nav2` para cerrar el ciclo de navegacion autonoma.

## Licencia
Este trabajo se distribuye bajo la licencia BSD-3-Clause (ver `autonomous_robot/LICENSE` y `autonomous_robot_description/LICENSE`).
