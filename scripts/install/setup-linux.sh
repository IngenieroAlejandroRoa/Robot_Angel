#!/usr/bin/env bash
# ==============================================================
# Robot Angel - Instalador general para Linux (Ubuntu / ROS 2 Jazzy)
# Versión 2.0 - Mejorada con comprobaciones y fallos tolerables
# ==============================================================

set -e  # Detener en errores graves

# --- Colores ---
GREEN='\033[1;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
NC='\033[0m'

echo -e "${GREEN}🤖 Instalador Robot Angel - Linux (ROS 2 Jazzy)${NC}"
sleep 1

# --- Crear entorno virtual ---
VENV_PATH="$HOME/.venvs/robot-angel"
if [ ! -d "$VENV_PATH" ]; then
  echo -e "${YELLOW}📦 Creando entorno virtual Python...${NC}"
  python3 -m venv "$VENV_PATH"
fi
source "$VENV_PATH/bin/activate"

# --- Actualizar pip y dependencias ---
echo -e "${GREEN}🐍 Actualizando pip y dependencias Python...${NC}"
pip install --upgrade pip setuptools wheel
pip install -r "$(dirname "$0")/requirements.txt" || true

# --- Actualizar repositorios ---
echo -e "${GREEN}🔍 Verificando y actualizando repositorios...${NC}"
sudo apt update -y

# --- Instalar dependencias del sistema ---
echo -e "${GREEN}⚙️  Instalando dependencias base del sistema...${NC}"
sudo apt install -y git curl build-essential python3-colcon-common-extensions python3-rosdep

# --- Instalar ROS 2 Jazzy si no está ---
if ! command -v ros2 &> /dev/null; then
  echo -e "${YELLOW}⚙️  Instalando ROS 2 Jazzy (puede tardar unos minutos)...${NC}"
  sudo apt install -y ros-jazzy-desktop
fi
source /opt/ros/jazzy/setup.bash

# --- micro-ROS Agent ---
echo -e "${GREEN}🚀 Instalando micro-ROS Agent...${NC}"
if apt-cache search ros-jazzy-micro-ros-agent | grep -q "ros-jazzy-micro-ros-agent"; then
  echo -e "${YELLOW}📦 Instalando paquete binario micro-ROS Agent...${NC}"
  sudo apt install -y ros-jazzy-micro-ros-agent
else
  echo -e "${YELLOW}⚠️  Paquete no disponible. Compilando desde fuente...${NC}"
  mkdir -p ~/uros_ws/src
  cd ~/uros_ws/src

  # Clonar solo si no existe
  if [ ! -d "micro-ROS-Agent" ]; then
    git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git || true
  fi

  cd ~/uros_ws
  rosdep update
  rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

  # Dependencias Python requeridas
  echo -e "${GREEN}🐍 Instalando dependencias de compilación Python...${NC}"
  pip install --upgrade catkin_pkg empy setuptools colcon-common-extensions

  # Compilar micro-ROS Agent
  echo -e "${GREEN}🧱 Compilando micro-ROS Agent (esto puede tardar)...${NC}"
  colcon build --symlink-install || {
    echo -e "${RED}❌ Error en compilación del micro-ROS Agent${NC}"
    exit 2
  }

  echo -e "${YELLOW}📄 Añadiendo micro-ROS Agent al entorno permanente...${NC}"
  if ! grep -q "uros_ws/install/setup.bash" ~/.bashrc; then
    echo 'source ~/uros_ws/install/setup.bash' >> ~/.bashrc
  fi
fi

# --- ESP-IDF ---
echo -e "${GREEN}🔧 Configurando ESP-IDF...${NC}"
mkdir -p ~/tools
cd ~/tools

# Clonar la versión estable más reciente
if [ ! -d "esp-idf" ]; then
  echo -e "${YELLOW}📦 Clonando ESP-IDF (release/v5.3)...${NC}"
  if git ls-remote --exit-code https://github.com/espressif/esp-idf.git release/v5.3 &>/dev/null; then
    git clone -b release/v5.3 --recursive https://github.com/espressif/esp-idf.git
  else
    echo -e "${YELLOW}⚠️  Rama v5.3 no encontrada, usando 'master' como respaldo...${NC}"
    git clone --recursive https://github.com/espressif/esp-idf.git
  fi
fi

cd esp-idf

# Instalar dependencias Python y herramientas del IDF
./install.sh || {
  echo -e "${RED}⚠️  Advertencia: falló el instalador de ESP-IDF, intenta manualmente más tarde con './install.sh'${NC}"
}

# Añadir export permanente si no existe
if ! grep -q "esp-idf/export.sh" ~/.bashrc; then
  echo 'source ~/tools/esp-idf/export.sh' >> ~/.bashrc
fi


# --- Arduino CLI ---
echo -e "${GREEN}🔩 Instalando Arduino CLI...${NC}"
mkdir -p ~/tools/bin
cd ~/tools

if ! command -v arduino-cli &> /dev/null; then
  echo -e "${YELLOW}📦 Descargando Arduino CLI...${NC}"
  curl -fsSL https://downloads.arduino.cc/arduino-cli/arduino-cli_1.3.1_Linux_64bit.tar.gz | tar xz -C ~/tools/bin --strip-components 1
fi

if ! grep -q "tools/bin" ~/.bashrc; then
  echo 'export PATH="$HOME/tools/bin:$PATH"' >> ~/.bashrc
fi
# 🧩 Añadir export automático de ESP-IDF al bashrc (si no existe)
if ! grep -q "esp-idf/export.sh" ~/.bashrc; then
  echo "source ~/tools/esp-idf/export.sh >/dev/null 2>&1" >> ~/.bashrc
  echo "⚙️  Añadido export automático de ESP-IDF al ~/.bashrc"
fi

# --- Resumen final ---
echo -e "\n${GREEN}✅ Instalación completa de Robot Angel.${NC}"
echo -e "${YELLOW}➡️  Recomendado: reinicia la sesión o ejecuta:${NC}"
echo -e "   source ~/.bashrc"
echo -e "${YELLOW}➡️  Luego activa el entorno Python con:${NC}"
echo -e "   source ~/.venvs/robot-angel/bin/activate"
echo -e "${YELLOW}➡️  Y verifica la instalación con:${NC}"
echo -e "   ros2 run micro_ros_agent micro_ros_agent --help"
echo -e "\n${GREEN}🤖 Robot Angel listo para despegar.${NC}"

