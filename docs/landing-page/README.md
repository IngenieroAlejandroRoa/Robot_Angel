# Robot Angel Landing Page

Landing page oficial del proyecto Robot Angel IDE.

## 🚀 Despliegue en GitHub Pages

### Opción 1: Mover archivos a la raíz (Recomendado)

Para publicar como GitHub Page en la raíz de tu repositorio:

1. Copia los archivos de esta carpeta a la raíz del proyecto:
```bash
cp docs/landing-page/index.html ./
cp docs/landing-page/styles.css ./
cp docs/landing-page/script.js ./
```

2. Ve a la configuración de tu repositorio en GitHub:
   - Settings → Pages
   - Source: Deploy from a branch
   - Branch: `main` / `root`
   - Guarda los cambios

3. Tu sitio estará disponible en: `https://tu-usuario.github.io/RobotAngel/`

### Opción 2: Usar directorio /docs

1. Ve a la configuración de tu repositorio en GitHub:
   - Settings → Pages
   - Source: Deploy from a branch
   - Branch: `main` / `/docs`
   - Guarda los cambios

2. Tu sitio estará disponible en: `https://tu-usuario.github.io/RobotAngel/landing-page/`

### Opción 3: GitHub Actions (Avanzado)

Crea el archivo `.github/workflows/pages.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: ["main"]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4
      - name: Setup Pages
        uses: actions/configure-pages@v4
      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: 'docs/landing-page'
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

## 🎨 Personalización

### Actualizar Links

Busca y reemplaza en `index.html`:
- `tu-usuario` → tu nombre de usuario de GitHub
- URLs de repositorio
- Email de contacto

### Modificar Colores

Los colores están definidos en `styles.css` en la sección de variables CSS:

```css
:root {
    --color-accent-primary: #007acc;
    --color-accent-secondary: #0e639c;
    /* ... más colores */
}
```

### Agregar Secciones

El HTML está organizado en secciones semánticas. Puedes agregar nuevas secciones siguiendo el patrón:

```html
<section class="tu-seccion">
    <div class="container">
        <div class="section-header">
            <h2 class="section-title">Título</h2>
            <p class="section-subtitle">Subtítulo</p>
        </div>
        <!-- Tu contenido -->
    </div>
</section>
```

## 📱 Características

- ✅ **Responsive Design**: Optimizado para móvil, tablet y desktop
- ✅ **Estética Theia IDE**: Colores y estilos inspirados en Theia
- ✅ **Animaciones Suaves**: Transiciones y efectos visuales
- ✅ **SEO Optimizado**: Meta tags y estructura semántica
- ✅ **Accesibilidad**: ARIA labels y navegación por teclado
- ✅ **Performance**: Lazy loading y optimizaciones

## 🛠️ Desarrollo Local

Para probar localmente:

```bash
# Servidor simple con Python
python3 -m http.server 8000

# O con Node.js
npx http-server -p 8000

# Abre en el navegador
http://localhost:8000
```

## 📝 Estructura de Archivos

```
landing-page/
├── index.html      # Página principal
├── styles.css      # Estilos (inspirados en Theia)
├── script.js       # JavaScript interactivo
└── README.md       # Esta documentación
```

## 🎯 Secciones Incluidas

1. **Hero**: Presentación principal con CTA
2. **Stats**: Estadísticas del proyecto
3. **Features**: Características principales
4. **Architecture**: Diagrama de arquitectura
5. **Quick Start**: Guía de inicio rápido
6. **Hardware**: Placas soportadas
7. **Documentation**: Enlaces a documentación
8. **Contribute**: Cómo contribuir
9. **About**: Sobre el proyecto
10. **Footer**: Enlaces y información legal

## 🔗 Links Importantes

Actualiza estos enlaces en `index.html`:

- Repositorio GitHub
- Issues
- Discussions
- Documentación
- Email de contacto

## 📊 Analytics (Opcional)

Para agregar Google Analytics, agrega antes de `</head>`:

```html
<!-- Google tag (gtag.js) -->
<script async src="https://www.googletagmanager.com/gtag/js?id=G-XXXXXXXXXX"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());
  gtag('config', 'G-XXXXXXXXXX');
</script>
```

## 🎨 Paleta de Colores

Basada en Theia IDE:

- **Primario**: #007acc (Azul Theia)
- **Secundario**: #4ec9b0 (Verde/Turquesa)
- **Fondo**: #1e1e1e (Negro oscuro)
- **Texto**: #cccccc (Gris claro)
- **Acento**: #c586c0 (Púrpura)

## 🐛 Easter Egg

¡Hay un easter egg escondido! Prueba el código Konami en la página 😉

```
↑ ↑ ↓ ↓ ← → ← → B A
```

## 📄 Licencia

La landing page es parte del proyecto Robot Angel y está bajo licencia GPLv3.

---

**Desarrollado con ❤️ para la comunidad Robot Angel**
