#!/bin/bash

echo "=== COMPILANDO Y PROBANDO ROBOT DOCKER ==="
echo ""

# Limpiar compilación anterior
echo "1. Limpiando compilación anterior..."
rm -rf build/ install/ log/

# Compilar proyecto
echo "2. Compilando proyecto..."
colcon build

if [ $? -eq 0 ]; then
    echo "✅ Compilación exitosa"
else
    echo "❌ Error en compilación"
    exit 1
fi

# Source setup
echo "3. Cargando configuración..."
source install/setup.bash

echo "4. Verificando archivos de configuración..."
if [ -f "install/autonomous_robot_description/share/autonomous_robot_description/config/rosmaster_x3/simple_controllers.yaml" ]; then
    echo "✅ Archivo de configuración encontrado"
else
    echo "❌ Archivo de configuración no encontrado"
    exit 1
fi

echo ""
echo "=== COMPILACIÓN COMPLETA ==="
echo ""
echo "Para probar el robot:"
echo "1. Terminal 1: ros2 launch autonomous_robot_gazebo rosmaster_x3_docker.launch.py"
echo "2. Terminal 2: ros2 run autonomous_robot_bringup simple_teleop"
echo ""
echo "Controles: W=adelante, S=atrás, A=izquierda, D=derecha, SPACE=parar, Q=salir"