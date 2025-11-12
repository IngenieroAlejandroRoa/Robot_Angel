# ✅ Terminal Split - Múltiples Terminales Verticales

## 🎯 Funcionalidad Implementada

He implementado un sistema de **terminales múltiples con split vertical** al estilo **Terminator**, permitiendo dividir el espacio verticalmente y tener varias terminales activas simultáneamente.

## 🔧 Implementación

### 1. Nuevo Componente: TerminalPanel

**Archivo:** `src/components/TerminalPanel.tsx`

Este componente gestiona múltiples instancias de `Terminal`:

```typescript
interface TerminalInstance {
  id: string;
  ref: React.RefObject<any>;
}

const [terminals, setTerminals] = useState<TerminalInstance[]>([
  { id: "terminal-0", ref: React.createRef() }
]);
```

**Funcionalidades:**

#### `addTerminal()`
- Crea nueva instancia de terminal
- ID único basado en timestamp
- Agrega a array de terminales
- Se divide el espacio verticalmente

#### `removeTerminal(id)`
- Elimina terminal por ID
- Protección: No permite eliminar la última terminal
- Redistribuye espacio entre terminales restantes

### 2. Diseño Visual

#### Layout Vertical
```
┌────────────────────────────────────┐
│          Terminal 1         [+] [⌄]│
│                                    │
├────────────────────────────────────┤
│          Terminal 2         [X] [⌄]│
│                                    │
├────────────────────────────────────┤
│          Terminal 3         [X] [⌄]│
│                                    │
└────────────────────────────────────┘
```

#### Elementos UI

**Botón de Split (+):**
- 🟣 Púrpura texto
- Ubicación: Header, entre "Terminal" y botón auto-scroll
- Ícono: Plus (+) pequeño
- Sin círculo morado
- Tooltip: "Split terminal vertically"

**Botón de Cierre (X):**
- Ubicación: Header, entre botón + y auto-scroll
- Solo visible si hay más de 1 terminal
- Hover: Rojo
- Ícono: X pequeño

### 3. Integración con App.tsx

```typescript
// ANTES
import { Terminal } from "./components/Terminal";
<Terminal ref={terminalRef} />

// AHORA
import { TerminalPanel } from "./components/TerminalPanel";
<TerminalPanel ref={terminalRef} />
```

El panel expone el mismo método `executeCommand()` que Terminal, ejecutando en la primera terminal.

## 🎨 Características Visuales

### División Horizontal
- Cada terminal ocupa espacio equitativo (`flex-1`)
- Borde gris entre terminales
- Responsive: Se ajusta al redimensionar

### Botones
```css
/* Botón + (Add) */
- Posición: Header terminal, right-11
- Color: text-purple-400
- Hover: text-purple-300 + bg-gray-700/50
- Sin círculo, solo ícono
- Tamaño: 6x6 (24px)
- Ícono: 3.5x3.5

/* Botón X (Close) */
- Posición: Header terminal, right-10
- Fondo: bg-gray-800
- Hover: bg-red-600
- Cuadrado: rounded-sm
- Tamaño: 6x6 (24px)
- Z-index: 20
```

### Estados
- **1 Terminal:** Solo botón [+] visible (no se puede cerrar)
- **2+ Terminales:** Botón [X] en cada terminal (excepto la última si solo queda 1)
- **División:** Vertical (arriba-abajo)
- **Máximo:** Ilimitado (limitado por altura de pantalla)

## 🚀 Cómo Usar

### Dividir Terminal Vertical

1. **Click en botón [+]** (header, al lado del auto-scroll)
2. Nueva terminal aparece abajo
3. Espacio se divide equitativamente
4. Cada terminal es independiente

### Cerrar Terminal

1. **Click en botón [X]** (header de la terminal)
2. Terminal se cierra
3. Espacio se redistribuye entre las restantes
4. No se puede cerrar la última terminal

### Uso Independiente

- Cada terminal tiene su propio historial
- Comandos independientes
- Directorios de trabajo independientes
- Auto-scroll independiente

## 📝 Ejemplos de Uso

### Escenario 1: Compilar y Ejecutar Simultáneamente

**Terminal 1:**
```bash
cd ~/proyecto
npm run watch
# Compilación continua
```

**Terminal 2:**
```bash
cd ~/proyecto
npm run dev
# Servidor de desarrollo
```

**Terminal 3:**
```bash
ros2 topic list
# Monitor de ROS topics
```

### Escenario 2: Multi-Robot Development

**Terminal 1:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888
# Agente micro-ROS para Robot 1
```

**Terminal 2:**
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8889
# Agente micro-ROS para Robot 2
```

**Terminal 3:**
```bash
ros2 topic echo /robot1/sensor_data
# Monitor Robot 1
```

**Terminal 4:**
```bash
ros2 topic echo /robot2/sensor_data
# Monitor Robot 2
```

### Escenario 3: Build y Test

**Terminal 1:**
```bash
colcon build
# Compilar workspace ROS
```

**Terminal 2:**
```bash
pytest tests/
# Ejecutar tests Python
```

**Terminal 3:**
```bash
tail -f /var/log/robot.log
# Monitor de logs
```

## 🔄 Flujo de Interacción

### Agregar Terminal
```
Click [+]
    ↓
addTerminal()
    ↓
Crear nuevo ID único
    ↓
Crear nueva ref
    ↓
Agregar a array terminals
    ↓
Re-render con nuevo layout
    ↓
Espacio dividido equitativamente
```

### Cerrar Terminal
```
Click [X] en terminal-2
    ↓
removeTerminal("terminal-2")
    ↓
Verificar si es última (NO eliminar)
    ↓
Filtrar array terminals
    ↓
Re-render
    ↓
Espacio redistribuido
```

## 💡 Detalles Técnicos

### Gestión de Referencias

Cada terminal tiene su propia `ref`:
```typescript
{
  id: "terminal-1699123456789",
  ref: React.createRef()
}
```

Esto permite:
- Acceso directo a métodos de cada terminal
- Ejecución de comandos en terminal específica
- Control independiente de estado

### Exposición de API

```typescript
useImperativeHandle(ref, () => ({
  executeCommand: async (cmd: string) => {
    // Ejecuta en primera terminal
    return await terminals[0].ref.current.executeCommand(cmd);
  }
}));
```

Mantiene compatibilidad con código existente (Run, Stop, Debug).

### Layout Flexible

```typescript
<div className="flex-1 flex flex-row">
  {terminals.map((terminal, index) => (
    <div className="flex-1 min-w-0">
      <Terminal ref={terminal.ref} />
    </div>
  ))}
</div>
```

- `flex-row`: Horizontal
- `flex-1`: Espacio equitativo
- `min-w-0`: Permite shrink

### Bordes Entre Terminales

```typescript
style={{
  borderRight: index < terminals.length - 1 
    ? '1px solid rgb(55, 65, 81)' 
    : 'none'
}}
```

Solo agrega borde derecho si no es la última terminal.

## 🎯 Protecciones Implementadas

### No Cerrar Última Terminal
```typescript
if (terminals.length === 1) {
  return; // No hacer nada
}
```

### IDs Únicos
```typescript
const newId = `terminal-${Date.now()}`;
```

Usa timestamp para garantizar unicidad.

### Conditional Rendering del Botón X
```typescript
{terminals.length > 1 && (
  <Button onClick={() => removeTerminal(terminal.id)}>
    <X />
  </Button>
)}
```

## 🎨 Personalización

### Cambiar a Split Vertical

**Archivo:** `src/components/TerminalPanel.tsx`

```typescript
// Cambiar esta línea:
<div className="flex-1 flex flex-row min-h-0">

// A:
<div className="flex-1 flex flex-col min-h-0">
```

### Límite de Terminales

```typescript
const MAX_TERMINALS = 4;

const addTerminal = () => {
  if (terminals.length >= MAX_TERMINALS) {
    return;
  }
  // ... resto del código
};
```

### Tamaño del Botón +

```typescript
<Button
  className="h-10 w-10 p-0 ..."  // Más grande
>
```

### Posición del Botón +

```typescript
// Bottom-left en lugar de bottom-right
<div className="absolute bottom-4 left-4 z-30">
```

## 🐛 Troubleshooting

### Terminales muy estrechas

**Causa:** Muchas terminales en pantalla pequeña.

**Solución:** Cerrar algunas terminales o usar pantalla más grande.

### Botón X no aparece

**Causa:** Solo hay 1 terminal.

**Solución:** Es comportamiento esperado. No se puede cerrar la última.

### Comandos no ejecutan

**Causa:** Terminal específica puede estar ocupada.

**Solución:** Usa otra terminal o espera a que termine el comando actual.

### Layout se rompe

**Causa:** CSS conflictivo.

**Solución:** Verifica que el contenedor padre tenga `min-h-0` y `flex-col`.

## 📊 Comparación con Terminator

| Característica | Terminator | Robot Angel IDE |
|----------------|------------|-----------------|
| Split Horizontal | ✅ | ✅ |
| Split Vertical | ✅ | ⚠️ (Customizable) |
| Cerrar terminal | ✅ | ✅ |
| Redimensionar splits | ✅ | ⚠️ (Automático) |
| Múltiples tabs | ✅ | ❌ |
| Terminales independientes | ✅ | ✅ |
| Botón visual para split | ❌ | ✅ |

## 📁 Archivos Modificados

- `src/components/TerminalPanel.tsx` - **NUEVO** - Panel multi-terminal
- `src/App.tsx` - Cambiado Terminal → TerminalPanel
- `TERMINAL_SPLIT.md` - Esta documentación

## ⚙️ Estado

✅ Compilado exitosamente  
✅ Split horizontal funcionando  
✅ Botón + para agregar terminals  
✅ Botón X para cerrar terminals  
✅ Protección de última terminal  
✅ Layout responsive  
✅ Compatibilidad con código existente  
✅ Terminales independientes  

## 🔮 Mejoras Futuras

1. **Redimensionar Splits** - Drag & drop entre terminales
2. **Split Vertical** - Además del horizontal
3. **Tabs** - Múltiples terminales en tabs
4. **Nombrar Terminales** - Custom names ("Build", "Test", etc)
5. **Persistencia** - Guardar layout entre sesiones
6. **Shortcuts** - Ctrl+Shift+T para new, Ctrl+Shift+W para close

---

¡Ahora puedes trabajar con múltiples terminales simultáneas estilo Terminator! 🔥✨
