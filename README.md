# 🤖 Robot Angel IDE

<div align="center">

![Robot Angel Logo](./logo.png)

**IDE Open Source especializado en robótica educativa y profesional**

[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Node.js](https://img.shields.io/badge/Node.js-18+-green.svg)](https://nodejs.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://www.python.org/)
[![Theia](https://img.shields.io/badge/Theia-1.63-purple.svg)](https://theia-ide.org/)

[Características](#-características) • [Instalación](#-instalación) • [Documentación](./docs/) • [Contribuir](#-contribuir) • [🌐 Landing Page](https://tu-usuario.github.io/RobotAngel/)

</div>

---

## 📋 Tabla de Contenidos

- [Visión y Misión](#-visión-y-misión)
- [Origen del Proyecto](#-origen-del-proyecto)
- [Características](#-características)
- [Arquitectura](#️-arquitectura)
- [Tecnologías Utilizadas](#-tecnologías-utilizadas)
- [Requisitos](#-requisitos)
- [Instalación](#-instalación)
- [Inicio Rápido](#-inicio-rápido)
- [Estructura del Proyecto](#-estructura-del-proyecto)
- [Roadmap](#-roadmap)
- [Contribuir](#-contribuir)
- [Licencia](#-licencia)
- [Citación](#-citación)
- [Agradecimientos](#-agradecimientos)

---

## 🎯 Visión y Misión

### Visión

**Robot Angel busca democratizar la robótica** mediante un IDE libre, multiplataforma y modular que integre herramientas modernas, automatice la instalación de middlewares especializados (micro-ROS, MicroPython) en placas de bajo costo (ESP32, RP2040) y provea documentación pedagógica para acelerar el aprendizaje en educación STEM y proyectos profesionales.

### Misión

Reducir drásticamente la curva de aprendizaje en robótica proporcionando:
- **Un entorno completo y profesional** sin costos de licencia
- **Automatización de tareas complejas** que tradicionalmente requieren configuración manual
- **Acceso a tecnologías vigentes** aplicadas en la industria actual
- **Herramientas libres** que cualquier persona pueda ver, usar, modificar y distribuir

### Objetivo General

> **Desarrollar un IDE open source especializado en el desarrollo de robótica**

### Objetivos Específicos

1. ✅ Integrar el entorno **Theia** como núcleo del compilador y editor de código fuente
2. ✅ Establecer funciones de **carga automática** de middlewares (micro-ROS, MicroPython) en placas
3. ✅ Publicar el proyecto con **documentación completa** bajo licencia GPLv3

---

## 🌱 Origen del Proyecto

Robot Angel nace como respuesta a una problemática identificada en el ecosistema de desarrollo robótico:

### El Problema

- **Curva de aprendizaje pronunciada**: Los interesados en robótica enfrentan barreras técnicas complejas desde el inicio
- **Herramientas fragmentadas**: IDEs existentes como MATLAB, RDS o MRDS son cerrados, costosos o con versiones gratuitas limitadas
- **Configuración manual compleja**: La instalación de middlewares y herramientas requiere conocimientos avanzados de terminal y compilación
- **Restricción del acceso**: Los costos de licencia limitan el acceso a escuelas públicas y jóvenes aficionados

### La Solución

Robot Angel se desarrolla bajo la filosofía del **software libre**, inspirado en el movimiento que construyó Linux, Git y Arduino. El proyecto reconoce que:

> *"La robótica es un pilar fundamental para el desarrollo del pensamiento lógico y las habilidades del siglo XXI en la educación STEM"*

Por ello, Robot Angel se posiciona como una alternativa viable y accesible que:
- Elimina barreras económicas (100% gratuito y libre)
- Simplifica la configuración técnica (automatización en un clic)
- Acerca a estudiantes a tecnologías industriales actuales
- Fomenta la colaboración comunitaria (código abierto)

### Inspiración

El proyecto se inspira en iniciativas exitosas como:
- **Arduino**: Democratización de la electrónica y robótica educativa
- **Linux y Git**: Modelo de desarrollo colaborativo y libre
- **ROS (Robot Operating System)**: Estándares abiertos para robótica profesional

---

## ✨ Características

### 🎨 Interfaz y Editor

- **IDE basado en Theia 1.63**: Editor moderno similar a VS Code
- **Editor de código integrado**: Sintaxis highlighting para C/C++, Python, JavaScript, Java
- **Terminal integrado**: Ejecución directa de comandos sin salir del IDE
- **Monitor Serial funcional**: Comunicación bidireccional con placas en tiempo real
- **Sistema de archivos**: Navegador de proyectos con vista de árbol

### 🚀 Automatización y Tooling

- **Detección automática de placas**: Reconoce ESP32, Arduino Uno/Mega/Nano y compatibles
- **Upload en un clic**: Compila y sube código a la placa automáticamente
- **Selección de puerto serie**: Dropdown dinámico con actualización automática
- **Instalación de middlewares**: Scripts automáticos para micro-ROS y MicroPython
- **Gestión de baudrate**: Configuración flexible para comunicación serial

### 🎓 Enfoque Educativo

- **Plantillas listas para usar**: Ejemplos de control de actuadores, sensores y nodos micro-ROS
- **Documentación pedagógica**: Guías paso a paso y referencias técnicas
- **Soporte multi-lenguaje**: Python, C/C++, JavaScript, Java
- **Ejecución inmediata**: Botón Run para ejecutar código sin configuración

### 🔧 Soporte de Hardware

- **Microcontroladores**: ESP32, ESP32-S2, ESP32-S3, ESP32-C3
- **Arduino**: Uno, Mega, Nano y compatibles
- **Raspberry Pi**: Pico (RP2040)
- **Comunicación**: USB Serial (UART), autodetección de puertos

### 🌍 Compatibilidad

- **Linux**: Ubuntu 20.04+, Debian 11+, Fedora 35+
- **Windows**: 10 y 11 (soporte completo)
- **Arquitectura**: x64, ARM64

---

## 🏗️ Arquitectura

Robot Angel está construido con una arquitectura modular de 3 capas:

```
┌─────────────────────────────────────────────────────────────┐
│                    CAPA DE PRESENTACIÓN                     │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Theia IDE Frontend (React + TypeScript)             │  │
│  │  - Editor de código (Monaco Editor)                  │  │
│  │  - UI personalizada (Robot Angel UI Extension)       │  │
│  │  - Monitor Serial (React Component)                  │  │
│  │  - Selector de placas y puertos                      │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                   CAPA DE LÓGICA DE NEGOCIO                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Theia Backend (Node.js + TypeScript)                │  │
│  │  - Terminal Backend (ejecución de código)            │  │
│  │  - Board Manager Backend (detección de placas)       │  │
│  │  - Serial Backend (comunicación serial)              │  │
│  │  - RPC Services (JSON-RPC sobre WebSocket)           │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                   CAPA DE ACCESO A HARDWARE                 │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Robot Angel Python Backend                          │  │
│  │  - Board Manager (pyserial + detección USB)          │  │
│  │  - Arduino Uploader (arduino-cli wrapper)            │  │
│  │  - Serial Monitor (pyserial bidireccional)           │  │
│  │  - ESP-IDF Environment Manager                       │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                              ↕
┌─────────────────────────────────────────────────────────────┐
│                      HARDWARE FÍSICO                        │
│    ESP32 | Arduino | RP2040 | Sensores | Actuadores        │
└─────────────────────────────────────────────────────────────┘
```

### Flujo de Trabajo

```
Usuario escribe código
    ↓
Presiona "Upload"
    ↓
Frontend envía solicitud al Backend (RPC)
    ↓
Backend Node.js llama Python script
    ↓
Python detecta placa y puerto
    ↓
arduino-cli compila código
    ↓
esptool sube firmware a placa
    ↓
Monitor serial muestra salida
```

---

## 🛠️ Tecnologías Utilizadas

### Frontend
- **Framework**: [Theia IDE 1.63](https://theia-ide.org/) - IDE desktop basado en web
- **UI Library**: [React 18.3](https://react.dev/) - Componentes de interfaz
- **UI Components**: [Radix UI](https://www.radix-ui.com/) - Primitivas accesibles
- **Styling**: [Tailwind CSS](https://tailwindcss.com/) - Utilidades CSS
- **Editor**: [Monaco Editor](https://microsoft.github.io/monaco-editor/) - Editor de código (mismo de VS Code)
- **Icons**: [Lucide React](https://lucide.dev/) - Iconos modernos

### Backend
- **Runtime**: [Node.js 18+](https://nodejs.org/) - Servidor de aplicación
- **Language**: [TypeScript 5.4](https://www.typescriptlang.org/) - Tipado estático
- **RPC**: JSON-RPC sobre WebSocket - Comunicación Frontend-Backend
- **Build Tool**: [Webpack 5](https://webpack.js.org/) - Empaquetado

### Python Backend
- **Language**: [Python 3.10+](https://www.python.org/)
- **Serial**: [pyserial 3.5](https://pyserial.readthedocs.io/) - Comunicación serial
- **CLI**: [Click 8.1](https://click.palletsprojects.com/) - Interface de línea de comandos
- **UI CLI**: [Rich 14.0](https://rich.readthedocs.io/) - Salida terminal enriquecida

### Herramientas de Desarrollo
- **Arduino**: [arduino-cli](https://arduino.github.io/arduino-cli/) - Compilación y upload
- **ESP32**: [esptool](https://docs.espressif.com/projects/esptool/) - Flasheo de firmware
- **ESP-IDF**: Toolchain oficial de Espressif
- **Version Control**: Git + GitHub

### Middleware Soportados
- **micro-ROS**: Framework ROS 2 para microcontroladores
- **MicroPython**: Implementación de Python para embebidos

---

## 📦 Requisitos

### Sistema Operativo

- **Linux**: Ubuntu 20.04+, Debian 11+, Fedora 35+ o equivalente
- **Windows**: 10 o 11 (64-bit)

### Software Base

- **Git** ≥ 2.40
- **Node.js** 18 LTS o superior + **npm** 9+
- **Python** 3.10+ con pip

### Herramientas de Desarrollo (se instalan automáticamente)

- arduino-cli
- esptool.py
- pyserial

### Drivers de Hardware

- **Drivers USB-Serial**: CP210x, CH34x, FTDI (según tu placa)
- **Permisos de puerto** (Linux): Usuario en grupo `dialout`

### Recursos del Sistema

- **RAM**: 4 GB mínimo, 8 GB recomendado
- **Disco**: 2 GB para el IDE + 1 GB para toolchains
- **Procesador**: Dual-core a 2 GHz o superior

---

## 💻 Instalación

### Linux (Ubuntu/Debian)

```bash
# 1. Instalar dependencias del sistema
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv nodejs npm build-essential

# 2. Clonar el repositorio
git clone https://github.com/tu-org/RobotAngel.git
cd RobotAngel

# 3. Crear entorno virtual de Python
python3 -m venv ~/.venvs/robot-angel
source ~/.venvs/robot-angel/bin/activate

# 4. Instalar dependencias de Python
pip install -r requirements.txt

# 5. Agregar usuario al grupo dialout (para acceso a puertos serie)
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar para aplicar cambios

# 6. Instalar arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
mkdir -p ~/tools/bin
mv bin/arduino-cli ~/tools/bin/
export PATH="$HOME/tools/bin:$PATH"

# 7. Configurar arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install esp32:esp32 --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# 8. Navegar al IDE y construir
cd robot-angel-ide/theia-ide-1.63.200/applications/electron
npm install
npm run build

# 9. Ejecutar Robot Angel IDE
npm start
```

### Windows

```powershell
# 1. Instalar Node.js desde https://nodejs.org/
# 2. Instalar Python desde https://www.python.org/

# 3. Clonar el repositorio
git clone https://github.com/tu-org/RobotAngel.git
cd RobotAngel

# 4. Crear entorno virtual
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# 5. Instalar dependencias de Python
pip install -r requirements.txt

# 6. Descargar arduino-cli desde https://arduino.github.io/arduino-cli/
# Extraer en C:\tools\arduino-cli\

# 7. Configurar arduino-cli
arduino-cli config init
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install esp32:esp32 --additional-urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# 8. Construir y ejecutar
cd robot-angel-ide\theia-ide-1.63.200\applications\electron
npm install
npm run build
npm start
```

---

## ⚡ Inicio Rápido

### 1. Abrir Robot Angel IDE

```bash
# Activar entorno virtual
source ~/.venvs/robot-angel/bin/activate  # Linux
# o
.\.venv\Scripts\Activate.ps1  # Windows

# Iniciar IDE
cd robot-angel-ide/theia-ide-1.63.200/applications/electron
npm start
```

### 2. Probar con Arduino Blink

1. Conecta tu placa Arduino/ESP32 por USB
2. En el IDE, crea un nuevo archivo `blink.ino`
3. Copia este código:

```cpp
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
```

4. Haz clic en el **selector de placa** (esquina superior)
5. Selecciona tu placa detectada
6. Haz clic en el botón **Upload** (⬆️)
7. Observa el LED parpadear

### 3. Usar Monitor Serial

1. Haz clic en el botón **Serial Monitor** (🔌)
2. Selecciona el **puerto** correcto
3. Configura el **baudrate** (por defecto 9600)
4. Haz clic en **Connect**
5. Observa los mensajes en tiempo real
6. Envía comandos escribiendo en el campo de texto

---

## 📁 Estructura del Proyecto

```
RobotAngel/
├── 📄 README.md                          # Este archivo
├── 📄 LICENSE                            # Licencia GPLv3
├── 📄 requirements.txt                   # Dependencias Python
├── 📄 setup.py                           # Configuración de paquete Python
│
├── 📁 robot_angel/                       # Backend Python
│   ├── 📄 __init__.py
│   ├── 📄 __main__.py                   # Punto de entrada CLI
│   ├── 📁 cli/                          # Comandos CLI
│   │   ├── core.py                      # Comandos principales
│   │   ├── project.py                   # Gestión de proyectos
│   │   └── agent.py                     # Agente micro-ROS
│   ├── 📁 utils/                        # Utilidades
│   │   ├── board_manager.py            # Detección de placas
│   │   ├── arduino_uploader.py         # Upload a Arduino/ESP32
│   │   ├── serial_monitor.py           # Comunicación serial
│   │   ├── esp_idf_env.py              # Gestión ESP-IDF
│   │   └── system.py                   # Utilidades del sistema
│   └── 📁 config/                       # Configuraciones
│
├── 📁 robot-angel-ide/                  # IDE Theia
│   └── 📁 theia-ide-1.63.200/
│       ├── 📁 applications/
│       │   └── 📁 electron/             # Aplicación Electron
│       │       ├── package.json
│       │       └── src/
│       └── 📁 theia-extensions/
│           └── 📁 theia-ide-angel-ui-ext/  # Extensión personalizada
│               ├── 📁 src/
│               │   ├── 📁 browser/      # Frontend (React)
│               │   │   ├── angel-widget.tsx
│               │   │   ├── board-manager-service.ts
│               │   │   ├── serial-service.ts
│               │   │   └── terminal-service.ts
│               │   ├── 📁 node/         # Backend (Node.js)
│               │   │   ├── terminal-backend.ts
│               │   │   ├── board-manager-backend.ts
│               │   │   └── serial-backend.ts
│               │   └── 📁 components/   # UI Components
│               │       ├── CodeEditor.tsx
│               │       ├── TopToolbar.tsx
│               │       ├── SerialMonitor.tsx
│               │       └── Terminal.tsx
│               └── package.json
│
├── 📁 docs/                             # Documentación
│   ├── ESP32_UPLOAD_SYSTEM.md          # Sistema de upload
│   ├── ARCHITECTURE.md                  # Arquitectura del proyecto
│   ├── DEVELOPMENT.md                   # Guía de desarrollo
│   └── USER_GUIDE.md                    # Guía de usuario
│
├── 📁 scripts/                          # Scripts de instalación
│   └── 📁 install/
│       ├── setup_arduino.sh
│       └── setup_esp32.sh
│
└── 📁 Robot Angel.../                   # Documento académico
    ├── Proyecto.tex                     # Trabajo de grado (LaTeX)
    ├── Citas.bib                        # Referencias bibliográficas
    └── Images/                          # Imágenes del documento
```

### Componentes Clave

| Directorio | Descripción |
|------------|-------------|
| `robot_angel/` | Backend Python: detección hardware, upload, serial |
| `robot-angel-ide/` | IDE Theia con extensión personalizada |
| `theia-ide-angel-ui-ext/` | Extensión custom: UI, servicios, backends |
| `src/browser/` | Frontend React: componentes y servicios |
| `src/node/` | Backend Node.js: RPC y lógica de negocio |
| `src/components/` | Componentes React reutilizables |
| `docs/` | Documentación técnica y guías |

---

## 🗺️ Roadmap

### ✅ Completado (v0.1.0)

- [x] IDE basado en Theia funcional
- [x] Editor de código con sintaxis highlighting
- [x] Terminal integrado
- [x] Ejecución de Python, C++, JavaScript, Java
- [x] Detección automática de placas (ESP32, Arduino)
- [x] Upload automático con un clic
- [x] Monitor Serial bidireccional funcional
- [x] Selector dinámico de puertos
- [x] Sistema de archivos integrado

### 🚧 En Progreso (v0.2.0)

- [ ] Instalador con un clic de micro-ROS
- [ ] Instalador con un clic de MicroPython
- [ ] Biblioteca de ejemplos y plantillas
- [ ] Visualización de datos seriales (gráficos)
- [ ] Autocompletado mejorado (LSP)
- [ ] Depurador integrado (DAP)

### 🔮 Futuro (v1.0.0+)

- [ ] Paquetes de instalación (.deb, .rpm, .msi)
- [ ] Soporte para más placas (STM32, nRF52)
- [ ] Simulador integrado de circuitos
- [ ] Gestión de bibliotecas visual
- [ ] Telemetría y visualización en tiempo real
- [ ] Soporte para ROS 2 Desktop
- [ ] Marketplace de extensiones
- [ ] Documentación pedagógica completa
- [ ] Integración con plataformas educativas

---

## 🤝 Contribuir

¡Las contribuciones son bienvenidas! Robot Angel es un proyecto comunitario.

### Cómo Contribuir

1. **Fork** el repositorio
2. Crea una rama para tu feature: `git checkout -b feat/nueva-funcionalidad`
3. Haz commit de tus cambios: `git commit -m 'Agrega nueva funcionalidad'`
4. Push a la rama: `git push origin feat/nueva-funcionalidad`
5. Abre un **Pull Request**

### Guías

- **Código de Conducta**: Sé respetuoso y constructivo
- **Estilo de Código**: 
  - TypeScript: seguir convenciones de Theia
  - Python: PEP 8
  - Commits: mensajes descriptivos en español o inglés
- **Tests**: Asegúrate de que tu código no rompa funcionalidad existente
- **Documentación**: Actualiza la documentación si es necesario

### Áreas de Contribución

- 🐛 **Bug fixes**
- ✨ **Nuevas features**
- 📝 **Documentación**
- 🎨 **Mejoras de UI/UX**
- 🧪 **Tests**
- 🌍 **Traducciones**
- 📚 **Contenido educativo**

### Comunidad

- **Issues**: [GitHub Issues](https://github.com/tu-org/RobotAngel/issues)
- **Discussions**: [GitHub Discussions](https://github.com/tu-org/RobotAngel/discussions)
- **Correo**: aroaapa33136@universidadean.edu.co

---

## 📜 Licencia

Robot Angel está distribuido bajo la **GNU General Public License v3.0 (GPLv3)**.

Esto significa que:
- ✅ Puedes **usar** el software libremente
- ✅ Puedes **estudiar** cómo funciona y modificarlo
- ✅ Puedes **distribuir** copias
- ✅ Puedes **mejorar** el software y publicar mejoras
- ⚠️ **Debes** mantener la misma licencia en trabajos derivados
- ⚠️ **Debes** publicar el código fuente de trabajos derivados

Consulta el archivo [LICENSE](./LICENSE) para más detalles.

### ¿Por qué GPLv3?

La licencia GPL asegura que Robot Angel y todos sus derivados permanezcan **libres y abiertos**, manteniendo el espíritu de democratización de la robótica.

---

## 📚 Citación

Si utilizas Robot Angel en investigación, educación o proyectos académicos, por favor cita:

### BibTeX

```bibtex
@software{robot_angel_2025,
  title        = {Robot Angel: IDE Open Source Especializado en Robótica},
  author       = {Roa Aparicio, Alejandro},
  year         = {2025},
  month        = {8},
  url          = {https://github.com/tu-org/RobotAngel},
  version      = {0.1.0},
  license      = {GPL-3.0},
  institution  = {Universidad EAN},
  note         = {Proyecto de Grado - Ingeniería de Sistemas}
}

@thesis{roa_aparicio_2025_robot_angel,
  title        = {Robot Angel: Elaboración de un Entorno Integrado de Desarrollo Open Source Especializado en Robótica},
  author       = {Roa Aparicio, Alejandro},
  year         = {2025},
  school       = {Universidad EAN},
  type         = {Trabajo de Grado},
  address      = {Bogotá, Colombia},
  note         = {Dirigido por PhD. Luisa Fernanda Carvajal Diaz}
}
```

### APA 7

```
Roa Aparicio, A. (2025). Robot Angel: IDE open source especializado en robótica 
    (Versión 0.1.0) [Software]. https://github.com/tu-org/RobotAngel

Roa Aparicio, A. (2025). Robot Angel: Elaboración de un entorno integrado de 
    desarrollo open source especializado en robótica [Trabajo de grado, 
    Universidad EAN]. Repositorio institucional Universidad EAN.
```

---

## 🙏 Agradecimientos

### Inspiración y Fundamentos

- **Comunidad Open Source**: Por demostrar que el software libre funciona
- **Theia IDE**: Por proveer una base sólida y extensible
- **Arduino**: Por democratizar la electrónica y la robótica educativa
- **micro-ROS**: Por acercar ROS 2 a los microcontroladores
- **MicroPython**: Por hacer Python accesible en embebidos

### Soporte Académico

- **Universidad EAN**: Por el apoyo institucional
- **PhD. Luisa Fernanda Carvajal Diaz**: Directora del proyecto
- **Estudiantes y docentes**: Que validaron el proyecto

### Tecnologías

- Eclipse Foundation (Theia)
- Microsoft (Monaco Editor, TypeScript)
- React Team
- Node.js Foundation
- Python Software Foundation
- Espressif Systems (ESP32)
- Arduino Team

### Comunidad

Gracias a todos los que contribuyen con código, documentación, reportes de bugs,
sugerencias y difusión del proyecto. Robot Angel es posible gracias a ustedes.

---

<div align="center">

**Hecho con ❤️ para la comunidad STEM**

*Democratizando la robótica, un estudiante a la vez*

[⬆ Volver arriba](#-robot-angel-ide)

</div>
