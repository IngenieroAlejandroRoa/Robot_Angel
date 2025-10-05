#!/usr/bin/env bash
# ==============================================================
# Robot Angel - Instalación de micro-ROS Agent (desde fuente)
# ==============================================================
# Ubicación: ~/Desktop/RobotAngel/scripts/install/install-microros-agent.sh
# Este script:
#   1. Prepara un workspace en ~/uros_ws
#   2. Clona el repo micro-ROS Agent (rama jazzy)
#   3. Instala dependencias
#   4. Compila con colcon
# ==============================================================

set -e

echo "🤖 Instalando micro-ROS Agent en ~/uros_ws ..."

# 1. Activar ROS 2 Jazzy (asegúrate de tenerlo instalado)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS 2 Jazzy detectado"
else
    echo "❌ ROS 2 Jazzy no encontrado en /opt/ros/jazzy"
    echo "Instala ROS 2 Jazzy antes de continuar: https://docs.ros.org/en/jazzy/"
    exit 1
fi

# 2. Crear workspace
mkdir -p ~/uros_ws/src
cd ~/uros_ws

# 3. Clonar micro-ROS Agent
if [ ! -d "src/micro-ROS-Agent" ]; then
    echo "📥 Clonando micro-ROS Agent..."
    git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
else
    echo "🔄 Actualizando micro-ROS Agent..."
    cd src/micro-ROS-Agent && git pull && cd ../..
fi

# 4. Instalar dependencias
echo "📦 Instalando dependencias..."
sudo apt update
sudo apt install -y python3-colcon-common-extensions
rosdep update
rosdep install --from-paths src --ignore-src -y

# 5. Compilar
echo "⚙️ Compilando micro-ROS Agent..."
colcon build

# 6. Configurar entorno
echo "✅ Compilación finalizada."
echo "👉 Para usar el agente, ejecuta:"
echo "   source ~/uros_ws/install/setup.bash"
echo "   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"
