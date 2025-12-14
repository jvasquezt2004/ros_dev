# Paquete minibot

Minibot es un robot movil de configuracion ackermman simulado en gazebo utilizando el stack de navegacion Nav2.

## ⚙️ Especificaciones Técnicas

### 1. Cinemática y Físicas (`robot_main.xacro`)
El robot utiliza el plugin `gz-sim-ackermann-steering-system` para la simulación física.

* **Tipo de Tracción:** Trasera (Ruedas `rear_left` y `rear_right`).
* **Dirección:** Delantera (Joints `front_left_steer` y `front_right_steer`).
* **Límites de Dirección:** Máximo 0.6 radianes (~34°) a la izquierda/derecha.
* **Dimensiones:**
    * Chasis: 0.32m (largo) x 0.24m (ancho).
    * Batalla (Wheelbase): 0.32m.
    * Ancho de vía (Track width): 0.266m.
* **Odometría:** Calculada y publicada por Gazebo en `/odom`.

### 2. Sensores Integrados
* **Lidar 2D (`lidar.xacro`):**
    * Sensor: `gpu_lidar`.
    * Tópico: `/scan`.
    * Configuración: 360° de visión, rango de 10 metros, frecuencia 10Hz.
    * Ubicación: Montado sobre el chasis con offset vertical.
* **Cámara RGB (`camera.xacro`):**
    * Tópico: `/camera/image_raw`.
    * Resolución: 640x480.
    * FOV Horizontal: ~1.09 rad.

## 🗺️ Flujo de Trabajo: Mapeo y Navegación

El uso del robot se divide en dos fases: primero generar el mapa (SLAM) y luego usar ese mapa para navegar de forma autónoma (Nav2).

### Fase 1: Generación de Mapa (SLAM) + Teleoperación

**Terminal 1: Simulación y SLAM**
Inicia Gazebo, el robot, los puentes de ROS, SLAM Toolbox y RViz configurado para mapeo.
```bash
ros2 launch minibot sim_world_robot.launch.py \
  use_slam:=true \
  use_rviz:=true \
  headless:=false
```

Terminal 2: Control Manual Mueve el robot para explorar. Recuerda que al ser Ackermann, debes avanzar (i) o freba (k) mientras giras (u/o).

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Una vez tengas el mapa completo, guárdalo usando el panel de "SlamToolbox" en RViz o mediante comandos de 
```bash
ros2 run nav2_map_server map_saver -f <nombre_del_mapa>
```

### Fase 2: Navegación Autónoma

Terminal 1: Simulación Base Inicia solo el entorno y el robot.

```bash
ros2 launch minibot sim_world_robot.launch.py \
  use_slam:=false \
  use_rviz:=false \
  headless:=false
```

Terminal 2: Stack de Navegación Inicia Nav2 cargando el mapa. Asegúrate de cambiar la ruta del mapa en base a donde lo hayas guardado).

```bash
ros2 launch minibot sim_nav.launch.py map:=ruta_mapa.yaml
```

Terminal 3: Visualización Abre RViz con la configuración específica para visualizar y utilizar Nav2.

```bash
ros2 launch minibot visualize_nav.launch.py
```

Dentro de la nueva ventana de Rviz aparecera el Tf del robot, selecciona 2D pose estimate y da click en el Td del robot (No debe ser muy preciso pero de preferencia en el centro del robot), posteriormente selecciona 2D Goal pose y selecciona donde quiere ir, veras que aparece una flecha verde, indicando hacia donde terminara mirando el robot una vez que llegue a la cola de la flecha.

![Ejemplo de la flecha en Nav2 y Rviz](images/Ejemplo_flecha_nav2.png)

## Estructura de archivos
* **launch/sim_world_robot.launch.py**: Launcher maestro. Orquesta Gazebo, el robot y (opcionalmente) SLAM.

* **launch/sim_nav.launch.p**y: Inicia el stack de Nav2 con RegulatedPurePursuitController.

* **launch/visualize_nav.launch.py**: Abre RViz con una configuracino para poder utilizar Nav2

* **config/minibot_slam_mapping.yaml**: Parámetros de SLAM Toolbox.

* **config/nav2_params.yaml**: Parámetros de navegación.
