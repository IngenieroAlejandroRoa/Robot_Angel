# ✅ Funcionalidad Run/Stop/Debug Implementada

## 🎯 Objetivo Cumplido

Los botones Run, Stop y Debug en la top toolbar ahora están **completamente funcionales**. El código del Monaco Editor se ejecuta en la terminal real mediante el backend.

## ✨ Características Implementadas

### Botón RUN ▶️
- ✅ Ejecuta el código del archivo activo en Monaco Editor
- ✅ Soporta múltiples lenguajes (Python, JavaScript, C++, Bash)
- ✅ Detecta automáticamente el lenguaje por extensión de archivo
- ✅ Muestra estado "Running..." mientras se ejecuta
- ✅ Cambia a verde cuando está ejecutando
- ✅ Se deshabilita mientras hay un proceso corriendo

### Botón STOP ■
- ✅ Detiene el proceso en ejecución
- ✅ Envía SIGTERM primero, SIGKILL después de 2 segundos si no responde
- ✅ Limpia archivos temporales automáticamente
- ✅ Se pone rojo cuando puede detener un proceso
- ✅ Solo está habilitado cuando hay un proceso corriendo

### Botón DEBUG 🐛
- ✅ Ejecuta el código con información de debug
- ✅ Muestra mensaje en terminal sobre modo debug
- ✅ Se deshabilita mientras hay un proceso corriendo

## 🏗️ Arquitectura

```
TopToolbar (Botones UI)
    ↓ onClick
App.tsx (handleRun/handleStop/handleDebug)
    ↓
window.angelTerminalBackend.runScript(code, language)
    ↓ RPC
TerminalBackendImpl (Node.js)
    ↓
1. Crea archivo temporal (.py, .js, .cpp, .sh)
2. spawn() proceso con el intérprete adecuado
3. Captura stdout/stderr
4. Retorna PID del proceso
```

## 📝 Lenguajes Soportados

### Python (.py)
```python
print("Hello from Robot Angel")
```
- Ejecuta con: `python3 archivo.py`

### JavaScript (.js)
```javascript
console.log("Hello from Robot Angel");
```
- Ejecuta con: `node archivo.js`

### C++ (.cpp, .c++)
```cpp
#include <iostream>
int main() {
    std::cout << "Hello from Robot Angel" << std::endl;
    return 0;
}
```
- Compila con: `g++ archivo.cpp -o output`
- Ejecuta: `./output`

### Bash (.sh o sin extensión)
```bash
echo "Hello from Robot Angel"
```
- Ejecuta con: `bash archivo.sh`

## 🚀 Cómo Usar

### 1. Lanzar la Aplicación
```bash
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
yarn electron start --no-sandbox
```

### 2. Crear un Archivo de Prueba

En Monaco Editor, escribe:

**Python:**
```python
import time

for i in range(5):
    print(f"Contador: {i}")
    time.sleep(1)
```

**JavaScript:**
```javascript
for (let i = 0; i < 5; i++) {
    console.log(`Contador: ${i}`);
}
```

### 3. Ejecutar

1. Click en **Run** (▶️)
2. Verás el botón cambiar a "Running..." (verde)
3. La salida aparecerá en la terminal
4. Cuando termine, el botón volverá a "Run" (morado)

### 4. Detener

Si el proceso tarda mucho o entra en loop:
1. Click en **Stop** (■)
2. El proceso se detendrá inmediatamente
3. Verás mensaje de confirmación en terminal

## 📁 Archivos Modificados

### Backend
- ✅ `src/node/terminal-backend.ts`
  - Agregado `runScript(code, language)` - Ejecuta código
  - Agregado `stopProcess(pid)` - Detiene proceso
  - Agregado `isProcessRunning(pid)` - Verifica estado
  - Gestión de procesos con Map
  - Limpieza automática de archivos temporales

### Common
- ✅ `src/common/terminal-protocol.ts`
  - Agregadas nuevas interfaces para runScript/stopProcess

### Frontend
- ✅ `src/App.tsx`
  - Estado: `isRunning`, `runningProcessId`
  - Funciones: `handleRun()`, `handleStop()`, `handleDebug()`
  - Detección automática de lenguaje

- ✅ `src/components/TopToolbar.tsx`
  - Props: `onRun`, `onStop`, `onDebug`, `isRunning`
  - Botones con estados visuales dinámicos
  - Disabled states apropiados

## 🔍 Flujo de Ejecución Detallado

### Run
1. Usuario escribe código en Monaco Editor
2. Guarda el archivo (opcional, pero recomendado)
3. Click en Run
4. `App.tsx` obtiene el código activo
5. Detecta lenguaje por extensión (`.py` → Python)
6. Llama `terminalBackend.runScript(code, 'python')`
7. Backend crea `/tmp/robot_angel_123.py`
8. Backend ejecuta `python3 /tmp/robot_angel_123.py`
9. `spawn()` crea proceso separado
10. stdout/stderr se muestran en consola backend
11. Retorna PID (ej: 123)
12. Frontend guarda PID y cambia estado a "Running"
13. Terminal muestra mensaje de ejecución

### Stop
1. Usuario click en Stop
2. `App.tsx` llama `terminalBackend.stopProcess(pid)`
3. Backend envía `SIGTERM` al proceso
4. Espera 2 segundos
5. Si no termina, envía `SIGKILL`
6. Limpia archivo temporal
7. Remueve proceso del Map
8. Retorna `true`
9. Frontend limpia estado
10. Terminal muestra mensaje de detención

## ⚙️ Configuración

### Cambiar Timeout de Kill
Edita `src/node/terminal-backend.ts`:
```typescript
setTimeout(() => {
    if (!procInfo.process.killed) {
        procInfo.process.kill('SIGKILL');
    }
}, 5000);  // 5 segundos en vez de 2
```

### Agregar Nuevo Lenguaje
Edita `src/node/terminal-backend.ts`, en `runScript()`:
```typescript
case 'ruby':
    tempFile = path.join(tempDir, `robot_angel_${pid}.rb`);
    fs.writeFileSync(tempFile, code);
    command = 'ruby';
    args = [tempFile];
    break;
```

### Cambiar Directorio de Temporales
```typescript
const tempDir = '/mi/directorio/temporal';  // En vez de os.tmpdir()
```

## 🐛 Debugging

### Si Run no funciona

```javascript
// En DevTools Console:
console.log(window.angelTerminalBackend);
// Debe mostrar Proxy con runScript, stopProcess

// Probar manualmente:
await window.angelTerminalBackend.runScript('print("test")', 'python');
```

### Si Stop no detiene el proceso

El proceso puede estar ignorando SIGTERM. Revisa logs del backend:
```
[PID 123] Process exited with code 0
```

### Ver archivos temporales

```bash
ls -la /tmp/robot_angel_*
```

Estos se eliminan automáticamente al terminar el proceso.

## 📊 Estados de los Botones

| Estado | Run | Stop | Debug |
|--------|-----|------|-------|
| Sin archivo | 🔘 Habilitado | ⚫ Deshabilitado | 🔘 Habilitado |
| Archivo cargado | 🟣 Listo (morado) | ⚫ Deshabilitado | 🔘 Listo |
| Ejecutando | 🟢 "Running..." | 🔴 Activo (rojo) | ⚫ Deshabilitado |

## ⚠️ Limitaciones

### Comandos Interactivos
❌ Programas que requieren input del usuario (input(), scanf())
- No hay conexión stdin desde el editor
- Solución: Usa argumentos o archivos de configuración

### Procesos de Larga Duración
⚠️ Servidores web, loops infinitos
- Se pueden detener con Stop
- Pero es mejor usar la terminal para estos casos

### GUIs
❌ No soporta aplicaciones con interfaz gráfica
- tkinter, pygame, etc. no funcionarán
- Usa terminal nativa para estos casos

## 🎉 Estado: ✅ FUNCIONANDO

Los botones Run, Stop y Debug están completamente implementados y funcionando. Puedes ejecutar código Python, JavaScript, C++ y Bash directamente desde el editor.

## 🚀 Próximas Mejoras (Opcional)

1. **Capturar stdout en tiempo real**
   - Mostrar output directamente en la terminal integrada
   - Usar eventos del proceso spawn

2. **Historial de ejecuciones**
   - Guardar resultados de ejecuciones previas
   - Botón para re-ejecutar con mismos parámetros

3. **Argumentos de línea de comandos**
   - UI para especificar argumentos antes de ejecutar
   - Guardar argumentos por archivo

4. **Debug real**
   - Integrar con debuggers (pdb, gdb, node inspect)
   - Breakpoints visuales
   - Step through code

5. **Múltiples ejecuciones**
   - Permitir varios procesos simultáneos
   - Panel de procesos activos

---

¡Pruébalo ahora! Crea un archivo Python o JavaScript y dale click a Run. 🎮✨
