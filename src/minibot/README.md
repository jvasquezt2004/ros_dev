<div align="center">

[![ES](https://img.shields.io/badge/Lang-ES-red)](README.md)
[![EN](https://img.shields.io/badge/Lang-EN-blue)](README_en.md)

</div>

# Paquete Minibot

Minibot es un robot móvil de configuración Ackermann simulado en Gazebo utilizando el stack de navegación Nav2.

## Especificaciones Técnicas

### 1. Cinemática y Física (`robot_main.xacro`)
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

## Flujo de Trabajo: Mapeo y Navegación

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

**Terminal 2: Control Manual**
Mueve el robot para explorar. Recuerda que al ser Ackermann, debes avanzar (`i`) o frenar (`k`) mientras giras (`u`/`o`).

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Una vez tengas el mapa completo, guárdalo usando el panel de "SlamToolbox" en RViz o mediante comandos de:

```bash
ros2 run nav2_map_server map_saver_cli -f <nombre_del_mapa>
```

### Fase 2: Navegación Autónoma

**Terminal 1: Simulación Base**
Inicia solo el entorno y el robot.

```bash
ros2 launch minibot sim_world_robot.launch.py \
  use_slam:=false \
  use_rviz:=false \
  headless:=false
```

**Terminal 2: Stack de Navegación**
Inicia Nav2 cargando el mapa. (Asegúrate de cambiar la ruta del mapa en base a donde lo hayas guardado).

```bash
ros2 launch minibot sim_nav.launch.py map:=ruta_mapa.yaml
```

**Terminal 3: Visualización**
Abre RViz con la configuración específica para visualizar y utilizar Nav2.

```bash
ros2 launch minibot visualize_nav.launch.py
```

Dentro de la nueva ventana de RViz aparecerá el TF del robot. Selecciona **2D Pose Estimate** y haz clic en el TF del robot (no debe ser muy preciso, pero de preferencia en el centro del robot). Posteriormente, selecciona **2D Goal Pose** y selecciona donde quieres ir; verás que aparece una flecha verde, indicando hacia donde terminará mirando el robot una vez que llegue a la cola de la flecha.

![Ejemplo de la flecha en Nav2 y Rviz](images/Ejemplo_flecha_nav2.png)

## Estructura de archivos

  * **launch/sim\_world\_robot.launch.py**: Launcher maestro. Orquesta Gazebo, el robot y (opcionalmente) SLAM.

  * **launch/sim\_nav.launch.py**: Inicia el stack de Nav2 con `RegulatedPurePursuitController`.

  * **launch/visualize\_nav.launch.py**: Abre RViz con una configuración para poder utilizar Nav2.

  * **config/minibot\_slam\_mapping.yaml**: Parámetros de SLAM Toolbox.

  * **config/nav2\_params.yaml**: Parámetros de navegación.

## Escenarios de Prueba Personalizados

### Mapa: Pasillo de Obstáculos
Se ha integrado un nuevo entorno de simulación diseñado para pruebas de navegación en espacios confinados.

* **Herramienta de Modelado:** Blender (Exportado a formato `.dae`/`.sdf`).
* **Geometría:**
    * **Tipo:** Pasillo recto (Corridor).
    * **Ancho de vía:** 1.0 metros (Distancia muro a muro), diseñado para probar la precisión del planificador local.
* **Obstáculos:** 4 elementos dinámicos/estáticos distribuidos en el trayecto:
    * 2 Cajas de madera.
    * 2 Barriles industriales.
* **Propósito:** Validar la capacidad del robot para maniobrar y evadir obstáculos sin colisionar en entornos estrechos donde el radio de giro Ackermann es crítico.

![Vista Superior del Pasillo](images/blender_model.png)
