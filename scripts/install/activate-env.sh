#!/usr/bin/env bash
# ==============================================================
# Robot Angel - Activar entornos
# ==============================================================
# Este script activa:
#  - Node.js 20 mediante NVM
#  - Python virtualenv "robot-angel"
# ==============================================================

echo "⚡ Activando entorno Robot Angel..."

# ---- Node (NVM) ----
if [ -s "$HOME/.nvm/nvm.sh" ]; then
    source "$HOME/.nvm/nvm.sh"
    nvm use 20 >/dev/null
    echo "✅ Node.js activo: $(node -v)"
    echo "✅ npm activo: $(npm -v)"
else
    echo "⚠️ NVM no encontrado. Verifica instalación."
fi

# ---- Python venv ----
VENV_PATH="$HOME/.venvs/robot-angel/bin/activate"
if [ -f "$VENV_PATH" ]; then
    # Forzar desactivar conda
    conda deactivate 2>/dev/null || true

    # Activar venv
    source "$VENV_PATH"

    echo "✅ Python venv activo: $(python3 --version)"
    echo "👉 Usando: $(which python3)"
else
    echo "⚠️ Virtualenv 'robot-angel' no encontrado."
fi

echo "🚀 Entorno listo para trabajar en Robot Angel"
