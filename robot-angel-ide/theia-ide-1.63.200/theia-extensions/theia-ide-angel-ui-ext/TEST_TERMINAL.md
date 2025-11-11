# 🔍 Guía de Pruebas - Terminal Embebida

## Objetivo
Verificar que la terminal nativa de Theia se embeba correctamente en el componente Terminal.tsx con la estética de Robot Angel.

## ✅ Pasos de Verificación

### 1. Iniciar la Aplicación
```bash
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
yarn electron start
```

### 2. Abrir DevTools
- Presiona `Ctrl+Shift+I` o `Cmd+Option+I`
- Ve a la pestaña **Console**

### 3. Verificar Logs Esperados

Deberías ver en la consola (en orden):

```
✅ AngelWidget init() called
✅ Terminal service exposed globally: [Object]
✅ AngelWidget calling update()
✅ AngelWidget render() called - rendering full App
✅ Waiting for terminal service... attempt 1
✅ Terminal service found, creating terminal...
✅ Terminal created successfully: [TerminalWidget]
✅ Terminal node found, embedding into container...
✅ Terminal embedded successfully
```

### 4. Verificar en el DOM

En la pestaña **Elements** de DevTools, busca:

```html
<div class="terminal-container" style="background-color: rgb(15, 15, 35);">
  <div class="theia-TerminalWidget">
    <div class="xterm">
      <!-- El terminal debe estar aquí -->
    </div>
  </div>
</div>
```

### 5. Verificar Funcionalidad

En el espacio de la terminal, deberías poder:
- ✅ Ver el prompt de tu shell
- ✅ Escribir comandos
- ✅ Ejecutar comandos (Enter)
- ✅ Ver el output
- ✅ Usar historial (↑ ↓)
- ✅ Auto-completar (Tab)

## 🧪 Comandos de Prueba

```bash
# Básicos
pwd
ls
echo "Hello Robot Angel"
date

# Interactivos
clear
top (presiona q para salir)

# Con colores
ls --color=auto
git status (si estás en un repo)
```

## ⚠️ Problemas Comunes y Soluciones

### Problema 1: "Waiting for service..." no avanza

**Síntoma:** El mensaje se queda en "Waiting for terminal service..."

**Solución:**
```javascript
// En la consola del navegador, verifica:
console.log(window.angelTerminalService);
// Debería mostrar un objeto, no undefined
```

Si es `undefined`:
1. Verifica que `AngelWidget.init()` se haya llamado
2. Revisa que no haya errores antes en la consola
3. Recarga la aplicación (Ctrl+R)

### Problema 2: Terminal node not available

**Síntoma:** Error "Terminal node not available"

**Solución:**
El widget de terminal no se creó correctamente. Verifica:
```javascript
// En consola:
window.angelTerminalService.ensureTerminal().then(t => {
  console.log('Terminal:', t);
  console.log('Terminal node:', t.node);
});
```

### Problema 3: Contenedor vacío (fondo oscuro pero sin terminal)

**Síntoma:** Ves el fondo oscuro (#0f0f23) pero no el terminal

**Solución:**
1. Verifica en Elements que el nodo del terminal esté dentro de `.terminal-container`
2. Busca estilos que oculten el terminal:
   ```javascript
   // En consola:
   const container = document.querySelector('.terminal-container');
   const termWidget = container.querySelector('.theia-TerminalWidget');
   console.log('Widget display:', getComputedStyle(termWidget).display);
   console.log('Widget visibility:', getComputedStyle(termWidget).visibility);
   ```

### Problema 4: Terminal se crea pero no se ve

**Síntoma:** Los logs dicen "Terminal embedded successfully" pero no ves nada

**Verifica el z-index y posicionamiento:**
```javascript
// En consola:
const container = document.querySelector('.terminal-container');
const termWidget = container.querySelector('.theia-TerminalWidget');
termWidget.style.position = 'relative';
termWidget.style.zIndex = '1';
termWidget.style.width = '100%';
termWidget.style.height = '100%';
```

## 🔧 Debug Avanzado

### Inspeccionar el Servicio de Terminal

```javascript
// En la consola del navegador:
const service = window.angelTerminalService;

// Ver la terminal actual
const terminal = service.getCurrentTerminal();
console.log('Current terminal:', terminal);

// Ver el nodo
console.log('Terminal node:', terminal?.node);

// Ver xterm
console.log('Xterm instance:', terminal?.term);
```

### Forzar Recreación del Terminal

```javascript
// Si algo salió mal, recrea el terminal manualmente:
window.angelTerminalService.ensureTerminal().then(term => {
  const container = document.querySelector('.terminal-container');
  container.innerHTML = '';
  container.appendChild(term.node);
  term.node.style.width = '100%';
  term.node.style.height = '100%';
  term.node.style.display = 'block';
});
```

## 📊 Checklist de Verificación

Después de iniciar, marca lo siguiente:

- [ ] Splash screen muestra logo Robot Angel
- [ ] Ventana dice "Robot Angel IDE"
- [ ] Widget de Angel UI está visible
- [ ] Terminal container tiene fondo oscuro (#0f0f23)
- [ ] Se ve el prompt del shell
- [ ] Puedo escribir en la terminal
- [ ] Los comandos se ejecutan correctamente
- [ ] El output se muestra
- [ ] Los colores ANSI funcionan (prueba con `ls --color`)
- [ ] El cursor es morado (#a855f7)
- [ ] El scrollbar es personalizado

## 📝 Reportar Problemas

Si algo no funciona, captura esta información:

1. **Logs de consola completos** (desde el inicio)
2. **Screenshot del área de terminal**
3. **Resultado de estos comandos en consola:**
   ```javascript
   console.log('Service:', window.angelTerminalService);
   console.log('Terminal:', window.angelTerminalService?.getCurrentTerminal());
   console.log('Container:', document.querySelector('.terminal-container'));
   ```

## 🎯 Resultado Esperado

Si todo funciona correctamente:
- ✅ Terminal completamente funcional
- ✅ Estética morada/oscura aplicada
- ✅ Header "Robot Angel Terminal"
- ✅ Sin errores en consola
- ✅ Comandos ejecutándose normalmente
