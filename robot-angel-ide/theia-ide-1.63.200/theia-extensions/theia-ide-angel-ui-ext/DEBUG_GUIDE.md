# Guía de Depuración - Terminal Integration

## Cambios Implementados para Captura de Output

### Problema Original
La terminal no mostraba la salida de los comandos ejecutados.

### Soluciones Implementadas

#### 1. **Captura Mejorada de Output** (`terminal-service.ts`)
- Ahora esperamos 500ms para que xterm se inicialice completamente
- Listener en `term.onData()` para capturar toda la salida
- Buffer de acumulación para procesar líneas completas
- Filtrado de líneas de prompt para evitar duplicados

#### 2. **Ejecución de Comandos Mejorada**
- Los comandos ahora se envían con `\r` (Enter) al final
- Limpieza del buffer antes de cada comando
- Logs de consola extensivos para depuración

#### 3. **Información de Contexto**
- Muestra el directorio actual en el prompt
- Ejecuta `pwd` automáticamente al iniciar
- Indicador visual del estado de conexión

## Cómo Probar

### 1. Abrir las DevTools de Electron
Presiona `Ctrl+Shift+I` o `Cmd+Option+I` para abrir las herramientas de desarrollo.

### 2. Ver la Consola
Busca estos mensajes en la consola:

```
AngelTerminalService initialized
Terminal service available, setting up...
Setting up terminal listeners, terminalId: xxx
Found xterm instance
Terminal connected and ready
Terminal ensured successfully
```

### 3. Ejecutar Comandos de Prueba

Prueba estos comandos en orden:

```bash
# 1. Ver directorio actual
pwd

# 2. Listar archivos
ls -la

# 3. Ver variables de entorno
echo $HOME

# 4. Comando simple
date

# 5. Ver contenido de un archivo
cat package.json
```

### 4. Verificar Output en Consola

Para cada comando ejecutado, deberías ver en la consola:

```
Executing command: pwd
Terminal data received: /home/user/path
Received history entry: {type: 'input', content: 'pwd'}
Received history entry: {type: 'output', content: '/home/user/path'}
```

## Problemas Comunes y Soluciones

### 1. No se ve ninguna salida

**Síntomas:**
- Los comandos se envían pero no aparece nada en el historial
- En consola: "Could not access xterm instance"

**Solución:**
```typescript
// Verifica en la consola del navegador:
// 1. ¿Se inicializa el servicio?
// 2. ¿Se encuentra la instancia de xterm?
// 3. ¿Se reciben eventos de data?

// Aumentar el delay de inicialización si es necesario:
await new Promise(resolve => setTimeout(resolve, 1000)); // En terminal-service.ts línea 63
```

### 2. Output duplicado o corrupto

**Síntomas:**
- Las líneas aparecen repetidas
- Caracteres extraños en la salida

**Solución:**
- Esto es normal con códigos ANSI de control
- Para mejorar, considera instalar `strip-ansi`:

```bash
npm install strip-ansi
```

Y úsalo en `processOutput()`:
```typescript
import stripAnsi from 'strip-ansi';

private processOutput(data: string): void {
    const cleanData = stripAnsi(data);
    // ... resto del código
}
```

### 3. Terminal no responde

**Síntomas:**
- No se ejecutan comandos
- En consola: "Terminal service not available"

**Solución:**
1. Verifica que el servicio esté inyectado correctamente
2. Comprueba los logs en `AngelWidget.init()`
3. Reinicia la aplicación

### 4. Prompt no muestra directorio correcto

**Síntomas:**
- Siempre muestra "$" genérico

**Solución:**
```typescript
// El método getCurrentDirectory() puede no estar disponible
// Como alternativa, parsea la salida de pwd:

setTimeout(() => {
    terminalServiceInstance.executeCommand('pwd');
}, 1000);
```

## Debugging Avanzado

### Inspeccionar el objeto Terminal

En la consola del navegador:

```javascript
// Obtener el servicio
const service = window.theia.container.get(/* AngelTerminalService ID */);

// Ver el terminal actual
const terminal = service.getCurrentTerminal();
console.log('Terminal:', terminal);

// Ver la instancia de xterm
console.log('Xterm:', terminal.term);

// Ver el buffer
console.log('Buffer:', terminal.term.buffer);
```

### Monitorear Eventos de Terminal

Agrega este código temporal en `setupTerminalListeners()`:

```typescript
// Log all possible events
terminalImpl.term.onData((data) => {
    console.log('[DATA]', data, data.charCodeAt(0));
});

terminalImpl.term.onLineFeed(() => {
    console.log('[LINE FEED]');
});

terminalImpl.term.onCursorMove(() => {
    console.log('[CURSOR MOVE]');
});
```

## Mejoras Futuras Recomendadas

1. **Integrar xterm-addon-fit** para mejor renderizado
2. **Usar xterm-addon-search** para búsqueda en historial
3. **Implementar ansi-to-react** para colores ANSI
4. **Añadir xterm-addon-web-links** para URLs clickeables
5. **Persistencia de historial** en localStorage

## Comandos Útiles de Depuración

```bash
# En el directorio de la extensión
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200/theia-extensions/theia-ide-angel-ui-ext

# Ver logs de compilación
npm run build 2>&1 | grep -i error

# Watch mode para desarrollo
npm run watch:theia

# En el directorio principal
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200

# Compilar en modo watch (en otra terminal)
yarn watch

# Ver archivos compilados
ls -la theia-extensions/theia-ide-angel-ui-ext/lib/browser/
```

## Contacto y Soporte

Si sigues teniendo problemas:
1. Revisa los logs en la consola del navegador
2. Verifica que `@theia/terminal` esté instalado
3. Comprueba que los archivos estén compilados en `lib/`
4. Intenta abrir la terminal nativa de Theia con el botón para comparar
