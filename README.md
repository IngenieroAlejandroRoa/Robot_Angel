# Robot Angel

> IDE open source especializado en robótica para educación y entornos profesionales. Reduce la curva de aprendizaje con instalación de *middlewares* (micro-ROS, MicroPython) en un clic, editor basado en Theia, plantillas para ESP32/RP2040 y documentación libre (GPLv3).

---

## 🧭 Tabla de contenidos

* [Visión](#-visión)
* [Características](#-características)
* [Arquitectura](#-arquitectura)
* [Requisitos](#-requisitos)
* [Instalación](#-instalación)

  * [Linux](#linux)
  * [Windows](#windows)
* [Inicio rápido](#-inicio-rápido)
* [Estructura del repositorio](#-estructura-del-repositorio)
* [Rutas de trabajo (Roadmap)](#-rutas-de-trabajo-roadmap)
* [Contribuir](#-contribuir)
* [Licencia](#-licencia)
* [Citación](#-citación)
* [Agradecimientos](#-agradecimientos)

---

## 🎯 Visión

Robot Angel busca **democratizar la robótica** mediante un IDE libre, multiplataforma y modular que integre *tooling* moderno (Theia, LSP/DAP), automatice la carga de *middlewares* (micro-ROS, MicroPython) en placas de bajo costo (ESP32, RP2040) y provea documentación pedagógica para acelerar prototipos en educación STEM y proyectos profesionales.

## ✨ Características

* **IDE basado en Theia**: editor moderno con soporte LSP/DAP, extensiones y vista de terminal integrada.
* **Instalación en un clic de *middlewares***: scripts para cargar **micro-ROS** y **MicroPython** en ESP32 / Raspberry Pi Pico.
* **Plantillas listas**: ejemplos de control de actuadores, sensores y nodos micro-ROS.
* **Compatibilidad OS**: validado en **Linux (Ubuntu/Debian)** y **Windows 10/11**.
* **Documentación abierta**: guías, manuales y referencias bajo **GPLv3**.
* **Enfoque educativo**: pensado para reducir la curva de aprendizaje en aulas, semilleros y *makerspaces*.

> Objetivo general: *Desarrollar un IDE open source especializado en desarrollo robótico.*

## 🧱 Arquitectura

* **Núcleo**: Theia (desktop/web) + extensiones para embebidos.
* **Automatización**: scripts (Python/Node) para instalar y flashear **micro-ROS** y **MicroPython**.
* **Targets**: ESP32, RP2040 (Raspberry Pi Pico).
* **Interfaz**: editor → terminal → tareas de construcción/flash → consola serie.
* **Documentación**: `/docs` con guías pedagógicas y de referencia.

```
(Usuario) ──▶ Robot Angel (Theia) ──▶ Scripts de carga ──▶ Placa (ESP32/RP2040)
                                  └─▶ Ejemplos / Plantillas ──▶ Sensores/Actuadores
```

## 🔧 Requisitos

* **Git** ≥ 2.40
* **Node.js LTS** + **npm** (Theia)
* **Python 3.10+** (scripts de automatización)
* **Drivers/Toolchains** según placa:

  * ESP32: ESP-IDF o toolchain Arduino-CLI (opcional), `esptool.py`.
  * RP2040: *bootloader* UF2 y herramientas de copia/flash.

## 🖥️ Instalación

### Linux

```bash
# 1) Clonar
git clone https://github.com/<org>/robot-angel.git
cd robot-angel

# 2) Dependencias del IDE
npm ci

# 3) Construir Theia (ejemplo)
npm run build

# 4) Ejecutar (modo escritorio/web)
npm run start
# o
npm run start:browser

# 5) Preparar scripts de automatización (opcional)
python3 -m venv .venv && source .venv/bin/activate
pip install -r tools/requirements.txt
```

### Windows

```powershell
# 1) Clonar
git clone https://github.com/<org>/robot-angel.git
cd robot-angel

# 2) Dependencias del IDE
npm ci

# 3) Build & Run
npm run build
npm run start

# 4) Scripts (PowerShell)
python -m venv .venv; .\.venv\Scripts\Activate.ps1
pip install -r tools/requirements.txt
```

> Recomendado: mantén actualizados los drivers USB–serial (CP210x/CH34x) para detección de puertos.

## ⚡ Inicio rápido

1. **Abrir Robot Angel** y seleccionar *plantilla*:

   * `examples/esp32/microros_blink/`
   * `examples/esp32/micropython_blink/`
   * `examples/rp2040/...`
2. **Conectar la placa** (verifica el puerto en la barra de estado).
3. **Flashear micro-ROS**:

   ```bash
   npm run flash:microros -- --port /dev/ttyUSB0
   ```
4. **Flashear MicroPython**:

   ```bash
   npm run flash:mpy -- --port /dev/ttyUSB0
   ```
5. **Abrir consola serie** integrada y validar salida.

## 🗂️ Estructura del repositorio

```
robot-angel/
├─ app/                  # Código del IDE (Theia + extensiones)
├─ examples/             # Plantillas para ESP32 / RP2040
│  ├─ esp32/
│  └─ rp2040/
├─ tools/                # Scripts de automatización (flash/install)
│  ├─ microros/
│  └─ micropython/
├─ docs/                 # Guías de uso, pedagogía, referencias
├─ .github/              # CI/CD, plantillas de issues/PRs
├─ LICENSE
└─ README.md
```

## 🗺️ Rutas de trabajo (Roadmap)

* [ ] Extensión Theia para **detectar placa** y puerto automáticamente.
* [ ] **Asistente de instalación** de micro-ROS/MicroPython (GUI).
* [ ] **Biblioteca de ejemplos** con sensores (LED, servos, cámara, WiFi/BLE).
* [ ] Integración básica de **visualización** (plots/telemetría).
* [ ] Paquetes para **Linux .deb** y **Windows MSI**.
* [ ] Documentación pedagógica (guías clase a clase y rúbricas).

## 🤝 Contribuir

1. Haz un **fork** y crea una rama: `feat/mi-feature`.
2. Ejecuta `npm test` / `lint` antes de enviar.
3. Abre un **Pull Request** con descripción clara (contexto, cambios, pruebas).
4. Revisa el **Código de Conducta** y la **Guía de Estilo** en `/docs`.

> Si eres docente/estudiante y quieres integrar Robot Angel en tu curso, revisa `/docs/edu/`.

## 📜 Licencia

Distribuido bajo **GNU GPLv3**. Consulta el archivo [`LICENSE`](./LICENSE).

## 📚 Citación

Si usas Robot Angel en investigación/enseñanza, por favor cita el proyecto y el documento del trabajo de grado:

```bibtex
@software{robot_angel,
  title        = {Robot Angel: IDE open source especializado en robótica},
  author       = {Roa Aparicio, Alejandro},
  year         = {2025},
  url          = {https://github.com/<org>/robot-angel}
}
```

## 🙏 Agradecimientos

* Comunidad **open source** (Theia, micro-ROS, MicroPython).
* Semilleros y docentes que apoyaron la validación y pruebas.

---

> **Nota**: Este README resume la propuesta académica: reducir la curva de entrada a robótica con herramientas libres, *scripts* automatizados y enfoque pedagógico. Cualquier persona puede **ver, usar, modificar y distribuir** el proyecto conforme a la GPLv3.
