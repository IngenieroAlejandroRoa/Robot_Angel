#!/bin/bash

# Robot Angel - Landing Page Deployment Script
# Este script facilita el despliegue de la landing page

set -e

echo "🤖 Robot Angel - Landing Page Deployment"
echo "========================================"
echo ""

# Colores
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Función para imprimir mensajes
print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# Verificar que estamos en el directorio correcto
if [ ! -d "docs/landing-page" ]; then
    print_error "Error: No se encuentra el directorio docs/landing-page"
    print_info "Asegúrate de ejecutar este script desde la raíz del proyecto"
    exit 1
fi

print_info "Directorio verificado"
echo ""

# Menú de opciones
echo "Selecciona una opción de despliegue:"
echo ""
echo "1) Desplegar en raíz del proyecto (recomendado)"
echo "2) Mantener en /docs/landing-page"
echo "3) Probar localmente"
echo "4) Configurar GitHub Pages"
echo "5) Cancelar"
echo ""
read -p "Opción [1-5]: " option

case $option in
    1)
        print_info "Copiando archivos a la raíz..."
        
        # Backup de archivos existentes si los hay
        if [ -f "index.html" ]; then
            print_warning "Existe index.html en la raíz. Creando backup..."
            cp index.html index.html.backup
            print_success "Backup creado: index.html.backup"
        fi
        
        # Copiar archivos
        cp docs/landing-page/index.html ./
        cp docs/landing-page/styles.css ./
        cp docs/landing-page/script.js ./
        
        print_success "Archivos copiados a la raíz"
        print_info "Ahora haz commit y push:"
        echo ""
        echo "  git add index.html styles.css script.js"
        echo "  git commit -m 'Add landing page'"
        echo "  git push origin main"
        echo ""
        print_info "Luego configura GitHub Pages:"
        echo "  Settings → Pages → Source: main branch / root"
        ;;
        
    2)
        print_info "Los archivos ya están en docs/landing-page"
        print_success "No es necesario mover archivos"
        print_info "Configura GitHub Pages:"
        echo "  Settings → Pages → Source: main branch / /docs"
        ;;
        
    3)
        print_info "Iniciando servidor local..."
        cd docs/landing-page
        
        if command -v python3 &> /dev/null; then
            print_success "Usando Python 3"
            print_info "Servidor en http://localhost:8000"
            echo ""
            python3 -m http.server 8000
        elif command -v python &> /dev/null; then
            print_success "Usando Python"
            print_info "Servidor en http://localhost:8000"
            echo ""
            python -m SimpleHTTPServer 8000
        elif command -v node &> /dev/null; then
            print_success "Usando Node.js"
            print_info "Instalando http-server..."
            npx http-server -p 8000
        else
            print_error "No se encontró Python ni Node.js"
            print_info "Instala Python 3 o Node.js para probar localmente"
            exit 1
        fi
        ;;
        
    4)
        print_info "Instrucciones para configurar GitHub Pages:"
        echo ""
        echo "1. Ve a tu repositorio en GitHub"
        echo "2. Click en 'Settings'"
        echo "3. En el menú lateral, click en 'Pages'"
        echo "4. En 'Source', selecciona:"
        echo "   - Branch: main"
        echo "   - Folder: /docs (o / root si copiaste a raíz)"
        echo "5. Click en 'Save'"
        echo "6. Espera unos minutos"
        echo "7. Tu sitio estará en: https://tu-usuario.github.io/RobotAngel/"
        echo ""
        print_success "¡Listo!"
        ;;
        
    5)
        print_info "Operación cancelada"
        exit 0
        ;;
        
    *)
        print_error "Opción inválida"
        exit 1
        ;;
esac

echo ""
print_success "Proceso completado"
