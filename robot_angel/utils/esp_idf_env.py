#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Módulo utilitario para garantizar que el entorno de ESP-IDF esté activo.
Si no está configurado, lo activa automáticamente cargando export.sh.
"""

import os
import shutil
import subprocess
from pathlib import Path

def ensure_esp_idf_env(verbose=True):
    """
    Asegura que el entorno de ESP-IDF esté activo.
    Devuelve True si está listo, False si falla.
    """
    idf_path = Path.home() / "tools" / "esp-idf"
    export_script = idf_path / "export.sh"

    if not export_script.exists():
        if verbose:
            print("❌ No se encontró ESP-IDF en ~/tools/esp-idf.")
        return False

    # Si ya está activo (idf.py disponible en PATH)
    if shutil.which("idf.py"):
        if verbose:
            print("✅ Entorno ESP-IDF ya activo.")
        return True

    if verbose:
        print("⚙️  Activando entorno ESP-IDF automáticamente...")

    activate_cmd = f"bash -c 'source {export_script} >/dev/null 2>&1 && env'"
    proc = subprocess.run(activate_cmd, shell=True, capture_output=True, text=True)

    if proc.returncode != 0:
        if verbose:
            print("❌ No se pudo cargar export.sh automáticamente.")
        return False

    # Cargar variables de entorno al proceso actual
    for line in proc.stdout.splitlines():
        key, _, value = line.partition("=")
        if key and value:
            os.environ[key] = value

    if verbose:
        print("✅ Entorno ESP-IDF activado correctamente.")
    return True
