# 🔍 Debugging Instructions - Robot Angel UI

## La extensión ahora tiene logs de debug activados

### Para ejecutar y ver los logs:

```bash
cd /home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200
yarn electron start
```

### Una vez que la aplicación se abra:

1. **Abre la consola del desarrollador**:
   - Presiona: `Ctrl + Shift + I`
   - O en el menú: `View` → `Toggle Developer Tools`

2. **Busca estos mensajes en la pestaña "Console"**:
   ```
   AngelWidgetContribution initializeLayout() called
   Main widgets found: X
   Closing widget: ...
   Opening Angel widget...
   AngelWidget init() called
   AngelWidget calling update()
   AngelWidget render() called
   Angel widget opened
   ```

### Qué esperar:

- **Si ves los logs**: El widget se está ejecutando, pero puede que el CSS lo esté ocultando
- **Si NO ves los logs**: El widget no se está inicializando correctamente

### Si ves los logs pero no la UI:

1. **Inspecciona el elemento**:
   - Click derecho en el área vacía
   - Selecciona "Inspect Element"
   - Busca un div con el texto "🤖 Robot Angel UI"

2. **Revisa el CSS**:
   - En el inspector, ve a la pestaña "Elements"
   - Busca elementos con `display: none` o `visibility: hidden`

### Si NO ves ningún log:

Significa que `initializeLayout()` no se está llamando. Posibles soluciones:

1. **Verifica que el comando manual funcione**:
   - En el menú: `View` → `Open Robot Angel UI`
   - Esto debería mostrar la UI

2. **Si el comando manual funciona**:
   - El problema está en `initializeLayout()`
   - Puede que necesitemos usar un approach diferente

## Versión actual (simplificada para debug):

He simplificado temporalmente el render para mostrar solo un mensaje de prueba:

```tsx
🤖 Robot Angel UI
Widget cargado exitosamente!
```

Una vez que confirmes que esto se muestra, restauraremos el componente App completo.

## Próximos pasos:

1. Ejecuta la app
2. Abre DevTools (Ctrl+Shift+I)
3. Busca los logs en la consola
4. Dime qué logs ves (o no ves)
5. Dime si ves la UI de prueba

