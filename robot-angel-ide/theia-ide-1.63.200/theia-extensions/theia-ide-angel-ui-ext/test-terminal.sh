#!/bin/bash

# Script de prueba rápida para la integración de terminal

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║       ROBOT ANGEL - Terminal Integration Test Script          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Colores
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

PROJECT_DIR="/home/ingeniero/Desktop/RobotAngel/robot-angel-ide/theia-ide-1.63.200"
EXT_DIR="$PROJECT_DIR/theia-extensions/theia-ide-angel-ui-ext"

echo "📁 Verificando estructura de archivos..."
echo ""

# Check source files
FILES=(
    "src/browser/terminal-service.ts"
    "src/hooks/useTerminalService.ts"
    "src/components/Terminal.tsx"
    "lib/browser/terminal-service.js"
    "lib/hooks/useTerminalService.js"
    "lib/components/Terminal.js"
)

cd "$EXT_DIR"

for file in "${FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file ${RED}(MISSING)${NC}"
    fi
done

echo ""
echo "📦 Verificando dependencias..."
echo ""

if grep -q '"@theia/terminal"' package.json; then
    echo -e "${GREEN}✓${NC} @theia/terminal en package.json"
else
    echo -e "${RED}✗${NC} @theia/terminal NO encontrado en package.json"
fi

echo ""
echo "🔨 Estado de compilación..."
echo ""

# Check if lib files are newer than src files
if [ "lib/browser/terminal-service.js" -nt "src/browser/terminal-service.ts" ]; then
    echo -e "${GREEN}✓${NC} Archivos compilados están actualizados"
else
    echo -e "${YELLOW}⚠${NC}  Los archivos fuente son más nuevos que los compilados"
    echo "   Ejecuta: npm run build"
fi

echo ""
echo "📝 Documentación disponible..."
echo ""

DOCS=(
    "TERMINAL_INTEGRATION.md"
    "DEBUG_GUIDE.md"
)

for doc in "${DOCS[@]}"; do
    if [ -f "$doc" ]; then
        echo -e "${GREEN}✓${NC} $doc"
    else
        echo -e "${YELLOW}⚠${NC}  $doc no encontrado"
    fi
done

echo ""
echo "🚀 Para iniciar la aplicación:"
echo ""
echo "   cd $PROJECT_DIR"
echo "   yarn electron start"
echo ""
echo "🔍 Para ver logs de depuración:"
echo ""
echo "   1. Abre la aplicación"
echo "   2. Presiona Ctrl+Shift+I (DevTools)"
echo "   3. Ve a la pestaña Console"
echo "   4. Busca mensajes de 'AngelTerminalService'"
echo ""
echo "📋 Comandos de prueba sugeridos en la terminal:"
echo ""
echo "   • pwd"
echo "   • ls -la"
echo "   • echo \$HOME"
echo "   • date"
echo "   • whoami"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
