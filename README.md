# Autonomous Robot Workspace

## Vision general
Este repositorio es un espacio de trabajo de ROS&nbsp;2 orientado al desarrollo de robots autónomos. Contiene dos proyectos principales:

1. **Robot Rosmaster X3**: Un robot omnidireccional basado en ruedas Mecanum para aplicaciones de navegación autónoma avanzada.
2. **Minibot**: Un robot educativo diferencial de dos ruedas diseñado para aprendizaje de conceptos básicos de robótica y ROS2.

El proyecto incluye descripciones URDF/Xacro completas, integración con Gazebo, configuraciones de visualización y archivos de lanzamiento para ambos robots.

> Nota: El workspace incluye el paquete `ros2_fundamentals_examples`, pensado únicamente como material educativo para aprender ROS. No forma parte del alcance del proyecto final y puede ignorarse salvo que se requieran ejemplos introductorios.

## Estructura del repositorio

### Robots principales

#### Autonomous Robot (Rosmaster X3)
- `autonomous_robot/`: Metapaquete que depende de la descripción del robot y servirá como agregador de nodos de alto nivel en el futuro.
- `autonomous_robot_description/`: Contiene los archivos Xacro/URDF, mallas STL, lanzadores y configuraciones para RViz y Gazebo del robot omnidireccional.
  - `urdf/robots/rosmaster_x3.urdf.xacro`: Ensambla la base, las ruedas, la cámara RGB-D, el lidar y la IMU.
  - `urdf/mech/`: Macros para la base y las ruedas Mecanum.
  - `urdf/sensors/`: Macros parametrizables para la cámara Intel RealSense D435, el lidar RPLidar S2 y una IMU genérica.
  - `meshes/`: Modelos 3D en formato STL para la visualización en RViz/Gazebo.

#### Minibot (Robot Educativo Diferencial)
- `minibot/`: Paquete Python completo con descripción URDF, lanzadores y configuraciones para un robot diferencial de dos ruedas.
  - `description/`: Archivos Xacro modulares del robot:
    - `robot.urdf.xacro`: Archivo principal que ensambla todos los componentes.
    - `robot_main.xacro`: Define el chassis, ruedas motrices y rueda loca.
    - `lidar.xacro`: Sensor lidar montado en la parte trasera.
    - `camera.xacro`: Cámara frontal para visión.
    - `inertial_macros.xacro`: Macros para cálculos de inercia.
  - `launch/`: Archivos de lanzamiento:
    - `display.launch.py`: Visualización en RViz2 con control de articulaciones.
  - `rviz/`: Configuraciones predefinidas de RViz2:
    - `display.rviz`: Configuración optimizada para visualización del minibot.

### Paquetes auxiliares
- `ros2_fundamentals_examples/`: Ejercicios de publicación/suscripción en Python y C++ (no usados en el proyecto final).

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

### Robot Rosmaster X3 (Autonomous Robot)

#### Generar el URDF expandido (a partir de Xacro)
```bash
ros2 run xacro xacro \
  $(ros2 pkg prefix autonomous_robot_description)/share/autonomous_robot_description/urdf/robots/rosmaster_x3.urdf.xacro \
  -o /tmp/rosmaster_x3.urdf
```

#### Visualizar en RViz2
```bash
ros2 launch urdf_tutorial display.launch.py model:=/tmp/rosmaster_x3.urdf
```
Esto abrirá RViz2 con `robot_state_publisher` y `joint_state_publisher_gui`. Ajusta las articulaciones de las ruedas desde la interfaz para comprobar el estado cinemático.

#### Cargar el robot en Gazebo Classic
```bash
# En una terminal
ros2 launch gazebo_ros gazebo.launch.py

# En otra terminal
ros2 run gazebo_ros spawn_entity.py \
  -file /tmp/rosmaster_x3.urdf \
  -entity rosmaster_x3
```
Los sensores publicados desde Gazebo utilizarán los tópicos definidos en los Xacro (`scan`, `imu/data`, `cam_1`).

#### Personalizar prefijos o instancias
El archivo principal `rosmaster_x3.urdf.xacro` acepta argumentos como `prefix` para instanciar múltiples robots y `use_gazebo` (reservado para futuras extensiones). Ejemplo:
```bash
ros2 run xacro xacro .../rosmaster_x3.urdf.xacro prefix:=robot1_ -o /tmp/robot1.urdf
```

### Minibot (Robot Educativo)

#### Visualización en RViz2
Para lanzar el minibot con visualización completa en RViz2:
```bash
# Asegúrate de estar en el workspace y tener el environment cargado
source install/setup.bash

# Lanza la visualización completa
ros2 launch minibot display.launch.py
```
Este comando iniciará:
- **robot_state_publisher**: Publica las transformaciones TF2 del robot
- **joint_state_publisher_gui**: Interfaz gráfica con sliders para controlar las ruedas
- **rviz2**: Visualización 3D con configuración predefinida

#### Opciones de lanzamiento del minibot
```bash
# Lanzar sin la interfaz gráfica de control de articulaciones
ros2 launch minibot display.launch.py gui:=false

# Generar archivo URDF expandido del minibot
ros2 run xacro xacro \
  $(ros2 pkg prefix minibot)/share/minibot/description/robot.urdf.xacro \
  -o /tmp/minibot.urdf

# Verificar el árbol de transformaciones
ros2 run tf2_tools view_frames
```

#### Tópicos y frames del minibot
El minibot publica y suscribe a los siguientes tópicos principales:
- `/joint_states`: Estados de las articulaciones de las ruedas
- `/robot_description`: Descripción URDF del robot
- `/tf` y `/tf_static`: Transformaciones entre frames

Jerarquía de frames:
```
base_link
├── base_footprint
├── chasis
│   ├── camera_link → camera_link_optical
│   └── lidar_frame
├── left_wheel
└── right_wheel
```

## Detalles tecnicos del robot URDF

### Robot Rosmaster X3 (Omnidireccional)

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

### Minibot (Robot Diferencial Educativo)

**Chassis y estructura**

| Parametro | Valor |
|-----------|-------|
| Longitud chassis (`chassis_length`) | 0.320 m |
| Anchura chassis (`chassis_width`) | 0.240 m |
| Altura chassis (`chassis_height`) | 0.078 m |
| Masa chassis | 0.700 kg |
| Material visual | Negro |
| Offset desde base_link | (-0.133, 0, -0.020) m |

**Ruedas motrices**

| Aspecto | Valor |
|---------|-------|
| Radio (`wheel_radius`) | 0.034 m |
| Grosor (`wheel_thickness`) | 0.026 m |
| Masa (cada rueda) | 0.050 kg |
| Separacion lateral (`wheel_offset_y`) | 0.266 m (centro a centro) |
| Offset vertical (`wheel_offset_z`) | 0.020 m |
| Articulaciones | `continuous`, eje Z (rotacion hacia atras) |
| Material visual | Naranja |
| Geometria de colision | Esfera (radio de rueda) |

**Rueda loca (caster)**

| Aspecto | Valor |
|---------|-------|
| Radio | 0.014 m (calculado automaticamente) |
| Masa | 0.010 kg |
| Offset longitudinal | 0.230 m (desde centro del chassis) |
| Offset vertical | Automatico para mantener contacto |
| Friccion Gazebo | `mu1=0.0001`, `mu2=0.0001` (muy baja) |
| Material visual | Naranja |

**Sensor Lidar**

| Parametro | Valor |
|-----------|-------|
| Tipo | GPU Lidar (Gazebo) |
| Radio visual | 0.025 m |
| Altura | 0.040 m |
| Posicion | (0.230, 0, 0.098) m desde chassis |
| Orientacion | 180° rotado (mirando hacia atras) |
| Rango | 0.08 - 10.0 m |
| Resolucion | 0.01 m |
| Muestras horizontales | 360 |
| Cobertura angular | 360° (-π a π rad) |
| Frecuencia | 10 Hz |
| Topico | `/scan` |
| Ruido | Gaussiano (σ=0.01) |

**Camara**

| Parametro | Valor |
|-----------|-------|
| Dimensiones | 0.01 x 0.03 x 0.03 m |
| Posicion | (0.318, 0, 0.063) m desde chassis |
| Orientacion | Mirando hacia adelante |
| Campo de vision horizontal | 1.089 rad (~62.4°) |
| Resolucion | 640x480 pixels |
| Formato | R8G8B8 |
| Rango de vision | 0.05 - 8.0 m |
| Frecuencia | 30 Hz |
| Topico | `/camera/image_raw` |
| Frame optico | `camera_link_optical` |

**Jerarquia de frames del minibot**
```
base_link (centro geometrico del robot)
├── base_footprint (proyeccion en el suelo)
├── chasis (offset hacia atras y abajo)
│   ├── camera_link → camera_link_optical (frente, arriba)
│   ├── lidar_frame (atras, arriba, rotado 180°)
│   └── caster_wheel (atras, en contacto con suelo)
├── left_wheel (izquierda, nivel de base_link)
└── right_wheel (derecha, nivel de base_link)
```

**Cinematica diferencial**
- Base de ruedas: 0.266 m
- Radio de ruedas: 0.034 m
- Configuracion: 2 ruedas motrices + 1 rueda loca trasera
- Control: Velocidades angulares independientes en ruedas izquierda y derecha
- Movimientos: Avance/retroceso, giro en el lugar, arcos de diferentes radios
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

### Para el Robot Rosmaster X3
1. Añadir lanzadores en `autonomous_robot_description/launch/` para automatizar RViz y Gazebo.
2. Incluir nodos de control en `autonomous_robot/` (p. ej. controlador de cinematica Mecanum, fusion de sensores).
3. Incorporar perfiles de `ros2_control` y `nav2` para cerrar el ciclo de navegacion autonoma.

### Para el Minibot
1. Implementar controlador de cinematica diferencial usando `ros2_control`.
2. Añadir launch files para simulacion en Gazebo.
3. Crear ejemplos de teleoperacion manual.
4. Implementar algoritmos basicos de navegacion (seguimiento de paredes, evasion de obstaculos).
5. Integrar con `nav2` para navegacion autonoma.

### Mejoras generales
1. Añadir tests automatizados para validar las descripciones URDF.
2. Documentar calibracion de sensores y parametros de simulacion.
3. Crear tutoriales paso a paso para cada robot.
4. Implementar configuraciones multi-robot para simulaciones colaborativas.

## Licencia
Este trabajo se distribuye bajo la licencia BSD-3-Clause (ver `autonomous_robot/LICENSE` y `autonomous_robot_description/LICENSE`).
