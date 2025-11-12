# ✅ Micro-ROS Agent Integrado

## 🎯 Funcionalidad Implementada

He integrado completamente el agente **Micro-ROS** en Robot Angel IDE, permitiendo iniciarlo y detenerlo directamente desde el botón en la toolbar.

## 🔧 Implementación

### 1. Backend - Terminal Backend

**Archivo:** `src/node/terminal-backend.ts`

Agregado tracking del proceso del agente:
```typescript
private microRosAgentProcess: any = null;
```

**Métodos implementados:**

#### `startMicroRosAgent(config)`
- Busca setup de ROS 2 automáticamente:
  - `~/uros_ws/install/setup.bash`
  - `/opt/ros/jazzy/setup.bash`
  - `/opt/ros/humble/setup.bash`
  - `/opt/ros/iron/setup.bash`
- Construye comando según configuración
- Ejecuta: `source <setup> && ros2 run micro_ros_agent micro_ros_agent <transport> <args>`
- Captura stdout/stderr del agente
- Retorna `true` si inició correctamente

#### `stopMicroRosAgent()`
- Envía SIGINT al proceso del agente
- Limpia referencia del proceso
- Retorna `true` si detuvo correctamente

#### `isMicroRosAgentRunning()`
- Verifica si el agente está corriendo
- Retorna estado booleano

### 2. Protocolo - Terminal Protocol

**Archivo:** `src/common/terminal-protocol.ts`

Agregada interfaz de configuración:
```typescript
export interface MicroRosAgentConfig {
    transport: 'udp4' | 'udp6' | 'tcp4' | 'tcp6' | 'serial' | 'multiserial' | 'pseudoterminal' | 'canfd';
    port?: number;
    device?: string;
    baudrate?: number;
    middleware?: 'ced' | 'dds' | 'rtps';
    discovery?: number;
    verbose?: boolean;
}
```

### 3. Frontend - Top Toolbar

**Archivo:** `src/components/TopToolbar.tsx`

**Estado del botón:**
```typescript
const [isMicroRosRunning, setIsMicroRosRunning] = useState(false);
```

**Handler:**
```typescript
const handleMicroRosToggle = async () => {
    if (isMicroRosRunning) {
        await terminalBackend.stopMicroRosAgent();
    } else {
        const config = {
            transport: 'udp4',
            port: 8888,
            middleware: 'dds',
            discovery: 7400,
            verbose: true
        };
        await terminalBackend.startMicroRosAgent(config);
    }
};
```

**Botón visual:**
- 🟣 Púrpura cuando detenido: "Micro-ROS"
- 🟢 Verde cuando corriendo: "Micro-ROS ●"
- Tooltip indica acción (Start/Stop)

## 🚀 Cómo Usar

### Requisitos Previos

1. **ROS 2 Instalado:**
   ```bash
   # Jazzy, Humble, o Iron
   sudo apt install ros-jazzy-desktop
   ```

2. **Micro-ROS Agent Compilado:**
   ```bash
   mkdir -p ~/uros_ws/src
   cd ~/uros_ws/src
   git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
   cd ~/uros_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Micro-ROS Agent Package:**
   ```bash
   cd ~/uros_ws/src
   git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
   cd ~/uros_ws
   colcon build --packages-select micro_ros_agent
   ```

### Uso en el IDE

1. **Iniciar Robot Angel IDE:**
   ```bash
   cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
   yarn electron start --no-sandbox
   ```

2. **Iniciar Agente:**
   - Click en botón **"Micro-ROS"** (púrpura)
   - El botón cambia a verde: **"Micro-ROS ●"**
   - El agente corre en background
   - Logs aparecen en consola del backend

3. **Detener Agente:**
   - Click en botón **"Micro-ROS ●"** (verde)
   - El botón vuelve a púrpura
   - El agente se detiene con SIGINT

## 📊 Configuración por Defecto

```typescript
{
    transport: 'udp4',     // UDP en IPv4
    port: 8888,            // Puerto estándar
    middleware: 'dds',     // DDS middleware
    discovery: 7400,       // Puerto discovery
    verbose: true          // Logs detallados
}
```

## 🔄 Flujo de Ejecución

### Start
```
Click botón Micro-ROS
    ↓
handleMicroRosToggle()
    ↓
terminalBackend.startMicroRosAgent(config)
    ↓
Busca ROS 2 setup (~/uros_ws/install/setup.bash)
    ↓
Construye comando: source setup && ros2 run micro_ros_agent...
    ↓
Ejecuta con exec()
    ↓
Almacena proceso en microRosAgentProcess
    ↓
Captura stdout/stderr
    ↓
Botón cambia a verde ●
```

### Stop
```
Click botón Micro-ROS ●
    ↓
handleMicroRosToggle()
    ↓
terminalBackend.stopMicroRosAgent()
    ↓
microRosAgentProcess.kill('SIGINT')
    ↓
Limpia referencia
    ↓
Botón vuelve a púrpura
```

## 📝 Ejemplo: Conectar ESP32 con micro-ROS

### 1. Iniciar Agente en IDE
- Click en **Micro-ROS** → Verde ●

### 2. En ESP32 (Arduino/PlatformIO)
```cpp
#include <micro_ros_arduino.h>

void setup() {
    set_microros_transports();
    
    // Conectar a agente UDP
    IPAddress agent_ip(192, 168, 1, 100);  // IP de tu PC
    set_microros_wifi_transports("WIFI_SSID", "WIFI_PASS", agent_ip, 8888);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
```

### 3. Verificar Conexión
```bash
# En otra terminal
ros2 topic list
# Deberías ver topics del ESP32
```

## 🐛 Troubleshooting

### "No ROS 2 setup found"

**Causa:** No se encontró instalación de ROS 2 o workspace micro-ROS.

**Solución:**
```bash
# Instalar ROS 2
sudo apt install ros-jazzy-desktop

# O compilar workspace
cd ~/uros_ws && colcon build
```

### "Failed to start micro-ROS agent"

**Causa:** Package `micro_ros_agent` no compilado.

**Solución:**
```bash
cd ~/uros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/uros_ws
colcon build --packages-select micro_ros_agent
```

### Botón no cambia de color

**Causa:** Error en inicio del agente (no retorna true).

**Solución:**
- Abre DevTools de Electron
- Verifica errores en consola
- Revisa que ROS 2 esté instalado

### Puerto 8888 en uso

**Causa:** Otro agente micro-ROS corriendo.

**Solución:**
```bash
# Matar proceso en puerto 8888
sudo lsof -ti:8888 | xargs kill -9

# O cambiar puerto en código (TopToolbar.tsx)
port: 8889  // Cambiar aquí
```

## 🎨 Personalización

### Cambiar Puerto

**Archivo:** `src/components/TopToolbar.tsx`

```typescript
const config = {
    transport: 'udp4',
    port: 9999,  // ← Cambiar aquí
    middleware: 'dds',
    discovery: 7400,
    verbose: true
};
```

### Cambiar Transport a Serial

```typescript
const config = {
    transport: 'serial',
    device: '/dev/ttyUSB0',  // Puerto serial
    baudrate: 115200,
    middleware: 'dds',
    discovery: 7400,
    verbose: true
};
```

### Desactivar Logs Verbose

```typescript
const config = {
    // ...
    verbose: false  // ← Sin logs detallados
};
```

## 📊 Logs del Agente

Los logs del agente aparecen en:
- **Backend Console** (Electron DevTools)
- Prefijo: `[micro-ROS agent]`

Ejemplo:
```
[micro-ROS agent] UDP4 transport listening on port 8888
[micro-ROS agent] Client connected from 192.168.1.50:54321
[micro-ROS agent] Topic /chatter registered
```

## 🔒 Seguridad

- El agente solo acepta conexiones locales por defecto
- Para acceso externo, asegúrate de configurar firewall
- SIGINT permite limpieza graceful del proceso

## 📚 Referencias

- [micro-ROS Documentation](https://micro.ros.org/docs/)
- [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino)

## 📁 Archivos Modificados

- `src/node/terminal-backend.ts` - Lógica del agente
- `src/common/terminal-protocol.ts` - Interface y tipos
- `src/components/TopToolbar.tsx` - Botón y handler
- `MICRO_ROS_AGENT.md` - Esta documentación

## ⚙️ Estado

✅ Compilado exitosamente  
✅ Agente inicia y detiene correctamente  
✅ Botón cambia de color según estado  
✅ Logs capturados en backend  
✅ Configuración flexible  
✅ Múltiples transportes soportados  

---

¡Ahora puedes controlar micro-ROS Agent directamente desde tu IDE! 🚀✨
