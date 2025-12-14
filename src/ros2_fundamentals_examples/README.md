<div align="center">

![ROS2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-D22128?style=for-the-badge&logo=opensourceinitiative&logoColor=white)

</div>

# Paquete: ROS 2 Fundamentals Examples 

Este paquete es una colección de ejemplos introductorios diseñados para aprender los conceptos básicos de **ROS 2** (Robot Operating System). No forma parte de la lógica del robot autónomo (Minibot), sino que sirve como material de estudio y prueba para verificar la instalación.

## Contenido del Paquete

El paquete incluye implementaciones paralelas en **C++** y **Python** de los patrones de comunicación más básicos.

| Lenguaje | Tipo | Código Fuente | Ejecutable | Tópico |
| :--- | :--- | :--- | :--- | :--- |
| **C++** | Publisher | `src/cpp_minimal_publisher.cpp` | `minimal_cpp_publisher` | `/cpp_example_topic` |
| **C++** | Subscriber | `src/cpp_minimal_subscriber.cpp` | `minimal_cpp_subscriber` | `/cpp_example_topic` |
| **Python** | Publisher | `.../py_minimal_publisher.py` | `py_minimal_publisher.py` | `/py_example_topic` |
| **Python** | Subscriber | `.../py_minimal_subscriber.py` | `py_minimal_subscriber.py` | `py_example_topic` |

## Compilación

Dado que es un paquete híbrido (`ament_cmake` con módulos de Python), se compila igual que el resto del workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_fundamentals_examples
source install/setup.bash
```

## Ejecución de Ejemplos

Asegúrate de tener **dos terminales** abiertas (una para publicar y otra para escuchar) y de haber cargado el entorno (`source install/setup.bash`) en ambas.

### 1\. Ejemplos en C++

**Terminal 1 (Publisher):**
Envía un mensaje de texto con un contador incremental cada 500ms.

```bash
ros2 run ros2_fundamentals_examples minimal_cpp_publisher
```

*Salida esperada:* `[INFO]: Publishing: 'Hello, world: 0'`

**Terminal 2 (Subscriber):**
Recibe e imprime los mensajes en la consola.

```bash
ros2 run ros2_fundamentals_examples minimal_cpp_subscriber
```

*Salida esperada:* `[INFO]: I heard: 'Hello, world: 0'`

### 2\. Ejemplos en Python

**Terminal 1 (Publisher):**
Envía un mensaje "Hello World" con un contador cada 0.5 segundos.

```bash
ros2 run ros2_fundamentals_examples py_minimal_publisher.py
```

**Terminal 2 (Subscriber):**
Escucha el tópico y muestra los datos recibidos.

```bash
ros2 run ros2_fundamentals_examples py_minimal_subscriber.py
```

## Inspección de Tópicos

Puedes usar las herramientas de CLI de ROS 2 para ver qué está pasando "detrás de cámaras" mientras corren los ejemplos:

```bash
# Ver la lista de tópicos activos
ros2 topic list

# Ver la frecuencia de publicación (hz)
ros2 topic hz /cpp_example_topic

# Escuchar manualmente el mensaje desde la terminal
ros2 topic echo /py_example_topic
```

### Detalles considerados:
1.  **Nombres de ejecutables:** Según tu `CMakeLists.txt`, los scripts de Python se instalan usando `install(PROGRAMS ...)`, lo que significa que mantienen su extensión `.py`. Por eso puse `py_minimal_publisher.py` en los comandos, a diferencia de C++ que compila a un binario sin extensión.
2.  **Tópicos:** Extraje los nombres de los tópicos directamente de tu código fuente: `/cpp_example_topic` para C++ y `/py_example_topic` para Python.
3.  **Badges:** Agregué los badges de C++ y Python para mantener el estilo visual consistente con los otros READMEs que hicimos.
