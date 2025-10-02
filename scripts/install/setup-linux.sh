#!/usr/bin/env bash
# ==============================================================
# Robot Angel - Setup Script for Ubuntu 24.04 (LTS)
# Autor: Alejandro Roa Aparicio
# Fecha: 2025-10-01
# ==============================================================

set -e  # detener en caso de error

echo "🚀 Iniciando instalación del entorno Robot Angel en Ubuntu 24.04..."

# -----------------------------
# 0. Crear estructura de carpetas base
# -----------------------------
echo "📂 Creando estructura de carpetas del proyecto..."

BASE_DIR="$HOME/Desktop/RobotAngel"

mkdir -p $BASE_DIR/{ide,extensions/{ra-dep-manager,ra-boards},scripts/{install,flash},drivers/udev,examples/{arduino,micropython,microros},docs}

# Archivos base
touch $BASE_DIR/.nvmrc
echo "20" > $BASE_DIR/.nvmrc
touch $BASE_DIR/.editorconfig
touch $BASE_DIR/README.md
touch $BASE_DIR/LICENSE

echo "✅ Carpetas creadas en: $BASE_DIR"

# -----------------------------
# 1. Paquetes base del sistema
# -----------------------------
echo "📦 Instalando paquetes base..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y git build-essential cmake ninja-build pkg-config \
    python3-venv python3-pip curl wget unzip zip tar \
    libx11-dev libxkbfile-dev libsecret-1-dev libusb-1.0-0-dev \
    libudev-dev make gcc g++ openssh-client

# -----------------------------
# 2. Node.js + Yarn (Theia/Electron)
# -----------------------------
echo "📦 Instalando Node.js (v20 LTS) con NVM..."
if [ ! -d "$HOME/.nvm" ]; then
  curl -fsSL https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
fi
export NVM_DIR="$HOME/.nvm"
source "$NVM_DIR/nvm.sh"

nvm install 20
nvm alias default 20

echo "📦 Instalando Yarn (classic)..."
npm i -g yarn@1

# -----------------------------
# 3. Python (virtualenv para Robot Angel)
# -----------------------------
echo "🐍 Configurando entorno Python..."
python3 -m venv ~/.venvs/robot-angel
source ~/.venvs/robot-angel/bin/activate
pip install --upgrade pip

# -----------------------------
# 4. Reglas de udev para placas (ESP32, Pico, CH340, CP210x, FTDI)
# -----------------------------
echo "🔌 Configurando reglas udev para acceso a placas..."
sudo usermod -a -G dialout $USER
sudo tee /etc/udev/rules.d/99-robot-angel-serial.rules >/dev/null <<'EOF'
# FTDI
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"
# Silicon Labs CP210x
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", MODE="0666", GROUP="dialout"
# QinHeng CH340/CH341
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"
# Raspberry Pi Pico (USB CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", MODE="0666", GROUP="dialout"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

# -----------------------------
# 5. ESP-IDF (toolchain ESP32)
# -----------------------------
echo "⚙️ Instalando ESP-IDF..."
mkdir -p ~/tools && cd ~/tools
if [ ! -d "esp-idf" ]; then
  git clone --recursive https://github.com/espressif/esp-idf.git
fi
cd esp-idf
./install.sh esp32,esp32s3
echo 'source ~/tools/esp-idf/export.sh' >> ~/.bashrc
source ~/tools/esp-idf/export.sh

# -----------------------------
# 6. MicroPython (esptool + mpremote)
# -----------------------------
echo "🐍 Instalando herramientas MicroPython..."
source ~/.venvs/robot-angel/bin/activate
pip install esptool mpremote

# -----------------------------
# 7. ROS 2 Jazzy + micro-ROS Agent
# -----------------------------
echo "🤖 Instalando ROS 2 Jazzy + micro-ROS Agent..."
sudo apt install -y ros-jazzy-ros-base ros-jazzy-micro-ros-agent || true
source /opt/ros/jazzy/setup.bash

# -----------------------------
# 8. Arduino CLI (opcional, como dependencia externa)
# -----------------------------
echo "🔧 Instalando Arduino CLI (opcional)..."
mkdir -p ~/tools && cd ~/tools
if [ ! -f "arduino-cli" ]; then
  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
fi
echo 'export PATH="$HOME/tools/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# -----------------------------
# 9. Mensaje final
# -----------------------------
echo "✅ Instalación y configuración inicial completas."
echo "📂 Proyecto listo en: $BASE_DIR"
echo "⚠️ IMPORTANTE: Reinicia la sesión para aplicar cambios de grupos (dialout)."
echo "👉 Para empezar a trabajar: activa el entorno Python con:"
echo "   source ~/.venvs/robot-angel/bin/activate"
