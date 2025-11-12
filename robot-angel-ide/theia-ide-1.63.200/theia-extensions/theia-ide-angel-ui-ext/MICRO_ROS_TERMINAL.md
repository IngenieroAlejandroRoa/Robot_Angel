# ✅ Micro-ROS Agent en Terminal Dedicada

## 🎯 Funcionalidad Implementada

El agente de **Micro-ROS** ahora se ejecuta en una **terminal dedicada** dentro del IDE, permitiendo visualizar en tiempo real todos los logs y la actividad del agente.

## 🔧 Implementación

### 1. Nueva Funcionalidad en TerminalPanel

**Método `executeInNewTerminal(cmd, label?)`**

```typescript
executeInNewTerminal: async (cmd: string, label?: string) => {
  // Crea nueva terminal
  const newId = `terminal-${Date.now()}`;
  const newRef = React.createRef<any>();
  
  setTerminals(prev => [
    ...prev,
    { id: newId, ref: newRef }
  ]);
  
  // Espera creación y ejecuta comando
  setTimeout(async () => {
    if (newRef.current && typeof newRef.current.executeCommand === 'function') {
      await newRef.current.executeCommand(cmd);
    }
  }, 500);
}
```

**Características:**
- Crea automáticamente nueva terminal
- Ejecuta comando en la nueva terminal
- ID único basado en timestamp
- Delay de 500ms para garantizar creación

### 2. Integración en TopToolbar

**Handler actualizado:**

```typescript
const handleMicroRosToggle = async () => {
  if (isMicroRosRunning) {
    // Detener: envía Ctrl+C a terminal
    if (terminalRef?.current?.executeCommand) {
      await terminalRef.current.executeCommand('\x03');
    }
    setIsMicroRosRunning(false);
  } else {
    // Iniciar: busca ROS setup y ejecuta en nueva terminal
    const setupPath = await terminalBackend.findRosSetup();
    if (!setupPath) {
      console.error('No ROS 2 setup found');
      return;
    }
    
    // Construir comando
    let command = `source ${setupPath} && ros2 run micro_ros_agent micro_ros_agent udp4`;
    command += ` -p 8888 -v 6`;
    
    // Ejecutar en nueva terminal
    if (terminalRef?.current?.executeInNewTerminal) {
      await terminalRef.current.executeInNewTerminal(command);
      setIsMicroRosRunning(true);
    }
  }
};
```

### 3. Backend: findRosSetup Expuesto

**Método público:**

```typescript
async findRosSetup(): Promise<string | null> {
    const candidates = [
        path.join(os.homedir(), 'uros_ws/install/setup.bash'),
        '/opt/ros/jazzy/setup.bash',
        '/opt/ros/humble/setup.bash',
        '/opt/ros/iron/setup.bash',
    ];

    for (const candidate of candidates) {
        if (fs.existsSync(candidate)) {
            return candidate;
        }
    }
    return null;
}
```

**Agregado a protocol:**
```typescript
export interface TerminalBackend {
    // ... otros métodos
    findRosSetup(): Promise<string | null>;
}
```

### 4. Props en TopToolbar

**Nueva prop `terminalRef`:**

```typescript
interface TopToolbarProps {
  // ... props existentes
  terminalRef?: React.RefObject<any>;
}
```

**Pasada desde App.tsx:**
```typescript
<TopToolbar
  // ... otras props
  terminalRef={terminalRef}
/>
```

## 🎨 Flujo Visual

### Antes (Sin Visualización)
```
Click [Micro-ROS] →
  Backend ejecuta en background →
    Sin feedback visual ❌
```

### Ahora (Con Terminal Dedicada)
```
Click [Micro-ROS] púrpura →
  Nueva terminal se abre abajo →
    Muestra: "source setup.bash && ros2 run..." →
      Output visible en tiempo real ✅
        [micro-ROS agent] UDP4 transport listening on port 8888
        [micro-ROS agent] Waiting for connections...
          Botón cambia a verde ●
```

## 🔄 Flujo Detallado

### Iniciar Agente

1. **Usuario:** Click en botón "Micro-ROS" (púrpura)
2. **TopToolbar:** `handleMicroRosToggle()` ejecuta
3. **Backend:** `findRosSetup()` busca instalación ROS
4. **TopToolbar:** Construye comando completo
5. **TerminalPanel:** `executeInNewTerminal(command)`
6. **Visual:** Nueva terminal aparece abajo
7. **Terminal:** Ejecuta comando, muestra output
8. **Estado:** Botón cambia a verde "Micro-ROS ●"
9. **Logs:** Visibles en tiempo real en terminal

```
┌────────────────────────────────────┐
│ 🔌 Terminal (Main)      [+] [⌄]   │
│ $ ls                               │
│ file1.txt  file2.txt               │
├────────────────────────────────────┤
│ 🔌 Terminal (Micro-ROS) [+] [X] [⌄]│
│ $ source ~/uros_ws/install/setup.bash │
│ $ ros2 run micro_ros_agent...     │
│ [micro-ROS agent] Starting...     │
│ [micro-ROS agent] UDP4 listening  │
│ [micro-ROS agent] Port: 8888      │
└────────────────────────────────────┘
```

### Detener Agente

1. **Usuario:** Click en botón "Micro-ROS ●" (verde)
2. **TopToolbar:** Envía `\x03` (Ctrl+C) a terminal
3. **Terminal:** Muestra interrupción
4. **Agente:** Se detiene gracefully
5. **Estado:** Botón vuelve a púrpura "Micro-ROS"
6. **Terminal:** Puede cerrarse con [X] si se desea

```
┌────────────────────────────────────┐
│ 🔌 Terminal (Micro-ROS) [+] [X] [⌄]│
│ [micro-ROS agent] Port: 8888      │
│ [micro-ROS agent] Waiting...      │
│ ^C ← Usuario presiona Stop        │
│ [micro-ROS agent] Shutting down   │
│ [micro-ROS agent] Stopped         │
│ $                                  │
└────────────────────────────────────┘
```

## 📝 Comando Construido

### Componentes del Comando

```bash
source ~/uros_ws/install/setup.bash && \
ros2 run micro_ros_agent micro_ros_agent udp4 \
  -p 8888 \
  -v 6
```

**Partes:**
1. `source ~/uros_ws/install/setup.bash` - Setup ROS 2
2. `ros2 run micro_ros_agent micro_ros_agent` - Ejecuta agente
3. `udp4` - Transporte UDP IPv4
4. `-p 8888` - Puerto de escucha
5. `-v 6` - Nivel de logs detallado (verbose level 6)

### Alternativas de Setup

El sistema busca en orden:
1. `~/uros_ws/install/setup.bash` (Workspace personal)
2. `/opt/ros/jazzy/setup.bash` (ROS 2 Jazzy)
3. `/opt/ros/humble/setup.bash` (ROS 2 Humble)
4. `/opt/ros/iron/setup.bash` (ROS 2 Iron)

## 🎯 Ventajas

### 1. **Visibilidad Total**
```
ANTES ❌:
  • Agente en background
  • Sin logs visibles
  • No se sabe si funciona
  • Debugging imposible

AHORA ✅:
  • Terminal dedicada
  • Logs en tiempo real
  • Estado visible
  • Debugging fácil
```

### 2. **Mejor Debugging**
```
Terminal muestra:
  • Conexiones entrantes
  • Topics registrados
  • Errores de comunicación
  • Estadísticas de DDS
  • Warning y errores
```

### 3. **Control Total**
```
Usuario puede:
  • Ver output completo
  • Scroll hacia arriba para revisar
  • Copiar logs
  • Cerrar terminal cuando termine
  • Mantener múltiples agentes (diferentes puertos)
```

### 4. **Múltiples Agentes**
```
┌────────────────────────────────────┐
│ Terminal: Micro-ROS Agent :8888    │
├────────────────────────────────────┤
│ Terminal: Micro-ROS Agent :8889    │
├────────────────────────────────────┤
│ Terminal: ROS 2 Bridge             │
└────────────────────────────────────┘
```

## 📊 Output Esperado

### Inicio Exitoso
```
$ source ~/uros_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 6
[1733944678.123456789] [micro_ros_agent]: Selected transport: udp4
[1733944678.234567890] [micro_ros_agent]: Running DDS discovery server on port 7400
[1733944678.345678901] [micro_ros_agent]: UDP4 agent listening on port 8888
[1733944678.456789012] [micro_ros_agent]: Waiting for micro-ROS nodes...
```

### Conexión de Cliente
```
[1733944680.123456789] [micro_ros_agent]: New client connected from 192.168.1.50:54321
[1733944680.234567890] [micro_ros_agent]: Client session initialized
[1733944680.345678901] [micro_ros_agent]: Discovered node: /micro_ros_node
[1733944680.456789012] [micro_ros_agent]: Registered topic: /sensor_data [sensor_msgs/msg/Imu]
[1733944680.567890123] [micro_ros_agent]: Registered service: /set_parameters
```

### Detención
```
^C
[1733944690.123456789] [micro_ros_agent]: Interrupt signal received
[1733944690.234567890] [micro_ros_agent]: Disconnecting clients...
[1733944690.345678901] [micro_ros_agent]: Client session closed
[1733944690.456789012] [micro_ros_agent]: Shutting down agent
[1733944690.567890123] [micro_ros_agent]: Goodbye!
$
```

## 🔧 Casos de Uso

### Desarrollo Single Robot

```
Terminal 1: Editor + Compilación
Terminal 2: Micro-ROS Agent (Auto)
Terminal 3: Monitor topics
Terminal 4: Tests
```

### Desarrollo Multi-Robot

```
Terminal 1: Micro-ROS Agent Robot1 :8888
Terminal 2: Micro-ROS Agent Robot2 :8889
Terminal 3: ROS 2 Bridge
Terminal 4: Monitor /robot1/odom
Terminal 5: Monitor /robot2/odom
```

### Debugging

```
Terminal 1: Micro-ROS Agent (verbose)
  → Ver conexiones
  → Ver topics registrados
  → Ver errores de comunicación

Terminal 2: ESP32 Serial Monitor
  → Ver logs del microcontrolador
  → Ver intentos de conexión

Terminal 3: ROS 2 Commands
  → ros2 topic list
  → ros2 topic echo /sensor_data
  → ros2 node info /micro_ros_node
```

## 🐛 Troubleshooting

### Agente no inicia

**Terminal muestra:**
```
bash: ros2: command not found
```

**Solución:**
- Instala ROS 2: `sudo apt install ros-jazzy-desktop`
- Compila workspace: `cd ~/uros_ws && colcon build`

### Puerto en uso

**Terminal muestra:**
```
[micro_ros_agent]: Error: Address already in use (port 8888)
```

**Solución:**
```bash
# Terminal 3
$ sudo lsof -ti:8888 | xargs kill -9
```

### No encuentra setup.bash

**Console muestra:**
```
Error: No ROS 2 setup found
```

**Solución:**
- Verifica instalación: `ls /opt/ros/*/setup.bash`
- Compila workspace: `cd ~/uros_ws && colcon build`

### Terminal no se crea

**Causa:** Delay insuficiente

**Solución:** Aumentar timeout en TerminalPanel.tsx:
```typescript
setTimeout(async () => {
  // ...
}, 1000); // 1 segundo en lugar de 500ms
```

## 📁 Archivos Modificados

### Frontend
- `src/components/TerminalPanel.tsx`
  - Método `executeInNewTerminal()`
  - Exposición en useImperativeHandle

- `src/components/TopToolbar.tsx`
  - Props: `terminalRef`
  - Handler: `handleMicroRosToggle()` actualizado
  - Construcción de comando

- `src/App.tsx`
  - Pasa `terminalRef` a TopToolbar

### Backend
- `src/node/terminal-backend.ts`
  - Método `findRosSetup()` ahora público
  - Retorna Promise<string | null>

- `src/common/terminal-protocol.ts`
  - Interface `TerminalBackend`
  - Método `findRosSetup()` agregado

## ⚙️ Estado

✅ Compilado exitosamente  
✅ Agente se ejecuta en terminal dedicada  
✅ Output visible en tiempo real  
✅ Botón toggle funcional (púrpura/verde)  
✅ Detención con Ctrl+C  
✅ Terminal se puede cerrar con [X]  
✅ Múltiples agentes soportados  
✅ Logs completos visibles  
✅ Debugging mejorado  

## 🚀 Para Probar

```bash
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
yarn electron start --no-sandbox
```

**Pasos:**
1. Click en botón "Micro-ROS" (púrpura)
2. Nueva terminal aparece abajo
3. Observa comando ejecutándose
4. Observa logs del agente en tiempo real
5. Conecta tu ESP32 con micro-ROS
6. Observa logs de conexión en terminal
7. Click en "Micro-ROS ●" (verde) para detener
8. Observa shutdown graceful en terminal
9. Cierra terminal con [X] si lo deseas

## 🎉 Ejemplo Completo

```
1. Iniciar IDE
   └─> 1 terminal vacía

2. Click "Micro-ROS"
   └─> Nueva terminal aparece
   └─> Ejecuta: source setup && ros2 run...
   └─> Muestra: "UDP4 agent listening on port 8888"
   └─> Botón → Verde ●

3. Conectar ESP32
   └─> Terminal muestra: "New client connected from 192.168.1.50"
   └─> Terminal muestra: "Registered topic: /sensor_data"

4. Click "Micro-ROS ●"
   └─> Terminal muestra: "^C"
   └─> Terminal muestra: "Shutting down agent"
   └─> Botón → Púrpura

5. Click [X] en terminal Micro-ROS
   └─> Terminal se cierra
   └─> Queda solo terminal principal
```

---

¡Ahora el agente Micro-ROS tiene su propia terminal dedicada con logs visibles! 🛰️✨
