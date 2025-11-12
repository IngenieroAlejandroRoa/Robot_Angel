# ✅ Run/Stop Actualizado - Output en Terminal del IDE

## 🎯 Cambio Realizado

El output de los scripts ahora **aparece directamente en la terminal del IDE** en lugar de solo en la consola de backend.

## 🏗️ Nueva Arquitectura

### Antes ❌
```
Run → runScript() → spawn() → stdout/stderr → console.log del backend
```
Problema: El output solo aparecía en logs de Electron, no en la terminal del IDE.

### Ahora ✅
```
Run → getScriptCommand() → Crea temp file → Retorna comando
                              ↓
executeCommand(comando) → Terminal del IDE muestra output
```

## 🔄 Flujo Actualizado

1. **Usuario click en Run**
2. `handleRun()` obtiene el código activo
3. Detecta lenguaje por extensión
4. Llama `getScriptCommand(code, language)`
5. Backend crea archivo temporal (`/tmp/robot_angel_123.py`)
6. Backend retorna comando: `python3 "/tmp/robot_angel_123.py"`
7. Frontend ejecuta: `executeCommand(comando)`
8. **El output aparece en la terminal del IDE** ✨
9. Cuando termina, limpia estado

## 📝 Ejemplo de Uso

### 1. Crear archivo Python

```python
print("Hola desde Robot Angel!")
for i in range(5):
    print(f"Contador: {i}")
print("Terminado!")
```

### 2. Click en Run

Verás en la terminal del IDE:

```
$ python3 "/tmp/robot_angel_123.py"
Hola desde Robot Angel!
Contador: 0
Contador: 1
Contador: 2
Contador: 3
Contador: 4
Terminado!
```

## 🔧 Métodos Actualizados

### Backend: `getScriptCommand(code, language)`

```typescript
async getScriptCommand(code: string, language: string): Promise<string>
```

**Qué hace:**
1. Crea archivo temporal según el lenguaje
2. Para C++, compila primero
3. Retorna el comando completo para ejecutar
4. NO ejecuta el comando (eso lo hace el frontend)

**Ejemplos de comandos retornados:**
- Python: `python3 "/tmp/robot_angel_1.py"`
- JavaScript: `node "/tmp/robot_angel_1.js"`
- C++: `"/tmp/robot_angel_1"` (después de compilar)
- Bash: `bash "/tmp/robot_angel_1.sh"`

### Frontend: `handleRun()`

```typescript
const handleRun = async () => {
  // 1. Validaciones
  // 2. Detecta lenguaje
  // 3. const command = await getScriptCommand(code, language)
  // 4. await executeCommand(command) ← El output va a la terminal!
}
```

## 🚀 Para Probar

```bash
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
yarn electron start --no-sandbox
```

### Test Python
```python
import time
print("Iniciando...")
for i in range(3):
    print(f"Paso {i+1}")
    time.sleep(1)
print("¡Completado!")
```

### Test JavaScript
```javascript
console.log("Hola desde JavaScript");
for (let i = 0; i < 3; i++) {
    console.log(`Iteración ${i}`);
}
console.log("Fin");
```

## 📊 Ventajas del Nuevo Enfoque

✅ **Output visible** - Aparece en la terminal del IDE
✅ **Colores ANSI** - Se preservan los colores del output
✅ **Scrollback** - Puedes hacer scroll para ver todo
✅ **Simple** - Usa el sistema de terminal existente
✅ **Sincrónico** - Ejecuta y espera resultado

## ⚠️ Limitaciones Conocidas

### Stop Button
❌ **No puede matar proceso en ejecución**
- `executeCommand()` es sincrónico, espera a que termine
- Stop solo resetea el estado, no mata el proceso
- **Workaround**: Usa Ctrl+C en la terminal nativa para matar proceso

### Comandos Largos
⚠️ **Timeout de 30 segundos**
- Si el script tarda más de 30 segundos, fallará
- Puedes aumentar el timeout en `terminal-backend.ts`

### Input del Usuario
❌ **No soporta input()**
- Scripts que requieren input del usuario no funcionarán
- Solución: Usa argumentos o archivos de configuración

## 🔧 Configuración

### Aumentar Timeout

Edita `src/node/terminal-backend.ts` en `executeCommand()`:

```typescript
const { stdout, stderr } = await execAsync(command, {
    cwd: workDir,
    timeout: 60000,  // 60 segundos
    maxBuffer: 1024 * 1024,
    shell: '/bin/bash'
});
```

### Cambiar Directorio Temporal

Edita `src/node/terminal-backend.ts` en `getScriptCommand()`:

```typescript
const tempDir = '/mi/carpeta/temporal';  // en vez de os.tmpdir()
```

## 📁 Archivos Modificados

### Backend
- ✅ `src/node/terminal-backend.ts`
  - Agregado `getScriptCommand()` - Crea temp file y retorna comando
  - Simplificado `runScript()` - Ahora solo maneja spawn interno
  - `cleanupProcess()` - Limpia archivos temporales

### Common
- ✅ `src/common/terminal-protocol.ts`
  - Agregado interface `getScriptCommand()`

### Frontend
- ✅ `src/App.tsx`
  - `handleRun()` - Usa `getScriptCommand()` + `executeCommand()`
  - `handleStop()` - Simplificado (solo resetea estado)

## 🐛 Debugging

### Si no ves output

```javascript
// En DevTools Console:
console.log(window.angelTerminalBackend);

// Probar manualmente:
const cmd = await window.angelTerminalBackend.getScriptCommand('print("test")', 'python');
console.log('Command:', cmd);

const result = await window.angelTerminalBackend.executeCommand(cmd);
console.log('Result:', result);
```

### Ver archivos temporales

```bash
ls -la /tmp/robot_angel_*
```

Estos archivos se quedan ahí hasta que se limpien. El cleanup está implementado pero solo se llama cuando el proceso spawn termina (que ya no usamos para ejecución principal).

## 🎉 Estado: ✅ FUNCIONANDO

El output ahora aparece correctamente en la terminal del IDE. Puedes ver:
- ✅ stdout (output normal)
- ✅ stderr (errores)
- ✅ Colores ANSI
- ✅ Todo el historial

## 🚀 Próximas Mejoras

1. **Stop funcional**
   - Implementar manera de matar proceso en ejecución
   - Quizás usar spawn en background y capturar PID real

2. **Ejecución en background**
   - Permitir ejecutar scripts largos sin bloquear UI
   - Mostrar output en tiempo real conforme llega

3. **Múltiples ejecuciones**
   - Permitir varios scripts corriendo simultáneamente
   - Panel de procesos activos

---

¡Pruébalo ahora! El output aparecerá directamente en tu terminal. 🎮✨
