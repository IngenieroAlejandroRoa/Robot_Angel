# 🤖 Contribuyendo a Robot Angel

¡Gracias por tu interés en contribuir a **Robot Angel** 🚀!  
Este proyecto busca democratizar la robótica a través de un IDE open source especializado.

---

## 📌 Flujo de trabajo con Git

1. **Ramas principales**
   - `main` → Rama estable. Solo versiones probadas y liberadas.
   - `dev` → Rama de desarrollo. Aquí se integran las nuevas funciones antes de pasar a `main`.

2. **Ramas de features**
   - Para cada nueva funcionalidad crea una rama desde `dev`:  
     ```bash
     git checkout dev
     git pull origin dev
     git checkout -b feature/nombre-feature
     ```
   - Ejemplo: `feature/gestor-dependencias`

3. **Commits**
   - Usa mensajes claros y cortos, preferiblemente con emojis:  
     - ✨ Nueva funcionalidad  
     - 🐛 Fix de bug  
     - 📝 Documentación  
     - 🎨 Cambios de estilo  
     - 🚀 Optimización  

   Ejemplo:
   ```bash
   git commit -m "✨ Añadido panel de configuración de placas"
