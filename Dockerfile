# /home/alonso/Development/School/ros_dev/Dockerfile

# 1. Empezar desde la imagen oficial de ROS 2 Jazzy con simulación
FROM osrf/ros:jazzy-desktop

# 2. Crear y establecer el directorio de trabajo dentro de la imagen
WORKDIR /root/ros2_ws

# 3. Copiar tu código fuente (solo la carpeta 'src') al directorio de trabajo
COPY ./src ./src

# 4. Actualizar, INICIALIZAR ROSDEP (ELIMINADO) e instalar todas las dependencias
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 5. (Opcional) Instalar tus paquetes extra que siempre usas
RUN apt-get update && apt-get install -y \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-mecanum-drive-controller \
    xterm \
    vim \
    && rm -rf /var/lib/apt/lists/*