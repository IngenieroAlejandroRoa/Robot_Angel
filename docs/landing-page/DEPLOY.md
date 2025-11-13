# Guía Rápida de Despliegue de Landing Page

## 🚀 Método Rápido (Recomendado)

Usa el script automatizado:

```bash
./scripts/deploy-landing.sh
```

El script te guiará por el proceso de despliegue.

## 🔧 Método Manual

### 1. Preparar archivos

Copia los archivos a la raíz del proyecto:

```bash
cp docs/landing-page/index.html ./
cp docs/landing-page/styles.css ./
cp docs/landing-page/script.js ./
```

### 2. Hacer commit

```bash
git add index.html styles.css script.js
git commit -m "Add landing page"
git push origin main
```

### 3. Configurar GitHub Pages

1. Ve a tu repositorio en GitHub
2. Settings → Pages
3. Source: `main` branch, `/` (root)
4. Save

### 4. Esperar

GitHub Pages tardará unos minutos en construir y desplegar.

Tu sitio estará en: `https://tu-usuario.github.io/RobotAngel/`

## 🎨 Personalización

Antes de desplegar, actualiza en `index.html`:

1. Busca `tu-usuario` y reemplaza con tu usuario de GitHub
2. Actualiza el email de contacto
3. Verifica todos los enlaces

## 🧪 Probar Localmente

```bash
cd docs/landing-page
python3 -m http.server 8000
```

Abre http://localhost:8000 en tu navegador.

## 📱 Características

- ✅ Responsive (móvil, tablet, desktop)
- ✅ Estética Theia IDE
- ✅ SEO optimizado
- ✅ Animaciones suaves
- ✅ Easter egg incluido 😉

## 🐛 Solución de Problemas

### La página no se muestra

1. Verifica que GitHub Pages esté habilitado
2. Espera 5-10 minutos después del push
3. Revisa la pestaña Actions para ver si hay errores

### Enlaces rotos

Verifica que las rutas a logos e imágenes sean correctas:
- `../../logo.png` apunta a la raíz del proyecto
- Ajusta según tu estructura

### Dominio personalizado

1. Crea archivo `CNAME` en la raíz con tu dominio
2. Configura DNS en tu proveedor
3. Espera a que se propague (puede tardar 24-48h)

## 📚 Más Información

- [Documentación de GitHub Pages](https://docs.github.com/pages)
- [README completo](docs/landing-page/README.md)
- [Guía de personalización](docs/landing-page/README.md#-personalización)

---

¿Necesitas ayuda? Abre un issue en el repositorio.
