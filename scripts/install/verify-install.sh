#!/usr/bin/env bash
echo "🤖 Verificando entorno Robot Angel..."
echo ""

# --- Funciones de ayuda ---
ok()   { echo "✅ $1"; }
fail() { echo "❌ $1"; }

# --- 1. Verificar Python ---
if command -v python3 &>/dev/null; then
  PYV=$(python3 --version 2>&1)
  ok "Python detectado: $PYV"
else
  fail "Python no está instalado"
fi

# --- 2. Verificar entorno virtual ---
if [[ -d "$HOME/.venvs/robot-angel" ]]; then
  ok "Entorno virtual robot-angel detectado"
else
  fail "Entorno virtual robot-angel no encontrado"
fi

# --- 3. Verificar ROS 2 Jazzy ---
if command -v ros2 &>/dev/null; then
  ROS2_VERSION=$(ros2 -h 2>/dev/null | grep -o 'ros2.*commands' | head -n1)
  ok "ROS 2 Jazzy detectado ($ROS2_VERSION)"
else
  fail "ROS 2 Jazzy no detectado"
fi

# --- 4. Verificar micro-ROS Agent ---
MICRO_OUT=$(ros2 run micro_ros_agent micro_ros_agent --help 2>&1)
if echo "$MICRO_OUT" | grep -q "Usage: 'micro_ros_agent"; then
  ok "micro-ROS Agent operativo"
else
  fail "micro-ROS Agent no responde correctamente"
fi

# --- 5. Verificar ESP-IDF ---
if [[ -d "$HOME/tools/esp-idf" ]]; then
  IDF_VER=$(grep -m1 "set(IDF_VERSION" "$HOME/tools/esp-idf/CMakeLists.txt" 2>/dev/null | grep -oE "[0-9]+\.[0-9]+")
  if [[ -n "$IDF_VER" ]]; then
    ok "ESP-IDF detectado (v$IDF_VER)"
  else
    ok "ESP-IDF detectado"
  fi
else
  fail "ESP-IDF no encontrado"
fi

# --- 6. Verificar Arduino CLI ---
if command -v arduino-cli &>/dev/null; then
  ARDUINO_VER=$(arduino-cli version 2>/dev/null | head -n1)
  ok "Arduino CLI operativo ($ARDUINO_VER)"
else
  fail "Arduino CLI no detectado"
fi

# --- 7. Verificar workspace micro-ROS ---
if [[ -d "$HOME/uros_ws/src/micro-ROS-Agent" ]]; then
  ok "Workspace micro-ROS presente"
else
  fail "Workspace micro-ROS no encontrado"
fi

# --- 8. Verificar PATH ---
if echo "$PATH" | grep -q "$HOME/tools/bin"; then
  ok "PATH incluye tools/bin"
else
  fail "tools/bin no está en PATH"
fi

echo ""
echo "📋 Verificación finalizada."
echo "Si todas las líneas son ✅, tu instalación está completamente funcional."
