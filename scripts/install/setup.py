#!/usr/bin/env python3
"""
Robot Angel - Instalador universal
Detecta el sistema operativo y lanza el script apropiado.
"""
import platform, subprocess, sys, os

system = platform.system()

try:
    if system == "Linux":
        print("🐧 Detectado Linux — ejecutando setup-linux.sh\n")
        subprocess.run(["bash", "scripts/install/setup-linux.sh"], check=True)
    elif system == "Windows":
        print("🪟 Detectado Windows — ejecutando setup-windows.ps1\n")
        subprocess.run([
            "powershell", "-ExecutionPolicy", "Bypass",
            "-File", "scripts\\install\\setup-windows.ps1"
        ], check=True)
    else:
        print(f"⚠️ Sistema operativo no soportado: {system}")
        sys.exit(1)
except subprocess.CalledProcessError as e:
    print(f"❌ Error durante la instalación: {e}")
    sys.exit(1)

print("\n✅ Instalación completa de Robot Angel.")
