# ✅ Checklist de Despliegue - Landing Page Robot Angel

## 📋 Lista de Verificación Pre-Despliegue

Marca cada item cuando lo completes:

### 1️⃣ Preparación de Archivos

- [ ] Clonar o estar en el repositorio Robot Angel
- [ ] Verificar que existe el directorio `docs/landing-page/`
- [ ] Verificar que existen los logos en la raíz:
  - [ ] `logo.png`
  - [ ] `logoVectorMejorado.svg`

### 2️⃣ Personalización

- [ ] Abrir `docs/landing-page/index.html` en un editor
- [ ] Buscar y reemplazar TODOS los `tu-usuario` con tu usuario de GitHub:
  ```bash
  sed -i 's/tu-usuario/TU_USUARIO_GITHUB/g' docs/landing-page/index.html
  ```
- [ ] Verificar email de contacto (línea ~550):
  - Email actual: `aroaapa33136@universidadean.edu.co`
  - [ ] Mantener o cambiar por tu email
- [ ] Revisar que todos los links apunten correctamente

### 3️⃣ Prueba Local

- [ ] Navegar a `docs/landing-page/`
- [ ] Iniciar servidor local:
  ```bash
  python3 -m http.server 8000
  ```
- [ ] Abrir navegador en `http://localhost:8000`
- [ ] Verificar:
  - [ ] Hero section se ve correctamente
  - [ ] Logos se cargan
  - [ ] Navegación funciona
  - [ ] Animaciones funcionan al hacer scroll
  - [ ] Botón de copiar código funciona
  - [ ] Links externos abren correctamente
  - [ ] Responsive design (probar en móvil)
- [ ] Detener el servidor (Ctrl+C)

### 4️⃣ Preparar para Despliegue

**Opción A: Desplegar en raíz (Recomendado)**

- [ ] Copiar archivos a la raíz:
  ```bash
  cp docs/landing-page/index.html ./
  cp docs/landing-page/styles.css ./
  cp docs/landing-page/script.js ./
  ```
- [ ] Configuración GitHub Pages: `main` / `/` (root)

**Opción B: Desplegar desde /docs**

- [ ] No mover archivos (ya están en `docs/landing-page/`)
- [ ] Configuración GitHub Pages: `main` / `/docs`

### 5️⃣ Git - Commit y Push

- [ ] Agregar archivos al staging:
  ```bash
  git add .
  ```
- [ ] Verificar cambios:
  ```bash
  git status
  ```
- [ ] Hacer commit:
  ```bash
  git commit -m "Add Robot Angel landing page with Theia-inspired design"
  ```
- [ ] Push al repositorio:
  ```bash
  git push origin main
  ```

### 6️⃣ Configurar GitHub Pages

- [ ] Ir a tu repositorio en GitHub
- [ ] Click en **Settings** (⚙️)
- [ ] Scroll down y click en **Pages** en el menú lateral
- [ ] En **Source**:
  - [ ] Branch: `main`
  - [ ] Folder: `/` (root) o `/docs` según tu elección
- [ ] Click en **Save**
- [ ] Esperar mensaje: "Your site is ready to be published at..."

### 7️⃣ Verificación Post-Despliegue

- [ ] Esperar 2-5 minutos para que GitHub Pages construya el sitio
- [ ] Visitar la URL: `https://tu-usuario.github.io/RobotAngel/`
- [ ] Verificar que la página carga correctamente
- [ ] Probar en diferentes dispositivos:
  - [ ] Desktop
  - [ ] Tablet
  - [ ] Móvil
- [ ] Verificar en diferentes navegadores:
  - [ ] Chrome/Chromium
  - [ ] Firefox
  - [ ] Safari (si disponible)

### 8️⃣ SEO y Compartir (Opcional)

- [ ] Verificar meta tags en la página (View Source)
- [ ] Probar compartir en redes sociales para ver preview
- [ ] Agregar link a la landing page en el README principal
- [ ] Configurar Google Analytics (opcional)
- [ ] Configurar dominio personalizado (opcional):
  - [ ] Crear archivo `CNAME` en la raíz
  - [ ] Configurar DNS en tu proveedor

### 9️⃣ Post-Despliegue

- [ ] Compartir en redes sociales
- [ ] Notificar a colaboradores
- [ ] Actualizar documentación con el link
- [ ] Crear un release en GitHub (opcional)

## 🐛 Troubleshooting

Si algo no funciona, verifica:

### Página no carga (404)
- [ ] GitHub Pages está habilitado en Settings → Pages
- [ ] Branch y folder correctos seleccionados
- [ ] Esperar más tiempo (puede tardar hasta 10 minutos)
- [ ] Verificar en Actions si hay errores de build

### Estilos no se aplican
- [ ] `styles.css` está en la misma carpeta que `index.html`
- [ ] Rutas en el `<link>` del HTML son correctas
- [ ] Limpiar caché del navegador (Ctrl+Shift+R)

### Imágenes rotas
- [ ] Logos existen en la raíz del repositorio
- [ ] Rutas son correctas: `../../logo.png`
- [ ] Verificar case-sensitive (logo.png vs Logo.png)

### Links no funcionan
- [ ] Todos los `tu-usuario` fueron reemplazados
- [ ] URLs tienen el formato correcto
- [ ] Links relativos vs absolutos correctos

## 📊 Métricas de Éxito

Una vez desplegado, verifica:

- [ ] Google PageSpeed Insights score > 90
- [ ] Tiempo de carga < 2 segundos
- [ ] Sin errores en la consola del navegador
- [ ] Todas las imágenes cargan
- [ ] Todos los links funcionan
- [ ] Responsive en móvil funciona bien

## 🎉 ¡Completado!

Si marcaste todos los items, ¡felicitaciones! 

Tu landing page de Robot Angel está en vivo en:
**https://tu-usuario.github.io/RobotAngel/**

---

## 📚 Recursos Adicionales

- [GitHub Pages Docs](https://docs.github.com/pages)
- [LANDING_PAGE_INSTRUCTIONS.md](./LANDING_PAGE_INSTRUCTIONS.md)
- [docs/landing-page/README.md](./docs/landing-page/README.md)
- [docs/landing-page/DEPLOY.md](./docs/landing-page/DEPLOY.md)

## 💬 ¿Necesitas Ayuda?

- 📧 Email: aroaapa33136@universidadean.edu.co
- 🐛 Issues: https://github.com/tu-usuario/RobotAngel/issues
- 💬 Discussions: https://github.com/tu-usuario/RobotAngel/discussions

---

**Última actualización**: 2025-11-13  
**Versión**: 1.0  
**Licencia**: GPLv3
