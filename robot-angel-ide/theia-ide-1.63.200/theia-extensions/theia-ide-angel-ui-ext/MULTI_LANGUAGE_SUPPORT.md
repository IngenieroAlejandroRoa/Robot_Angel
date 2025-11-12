# ✅ Soporte Multi-Lenguaje Expandido

## 🎯 Lenguajes Ahora Soportados

He agregado soporte para **11 lenguajes de programación** en total:

### ✅ Lenguajes Funcionando

| Lenguaje | Extensión | Ejecutor | Estado |
|----------|-----------|----------|--------|
| **Python** | `.py` | `python3` | ✅ Funcionando |
| **JavaScript** | `.js` | `node` | ✅ Funcionando |
| **TypeScript** | `.ts` | `node` | ✅ Funcionando |
| **C++** | `.cpp`, `.c++`, `.cc`, `.cxx` | `g++` | ✅ Funcionando |
| **C** | `.c` | `g++` | ✅ Funcionando |
| **Java** | `.java` | `javac` + `java` | ✅ NUEVO |
| **HTML** | `.html`, `.htm` | Browser | ✅ NUEVO |
| **PHP** | `.php` | `php` | ✅ NUEVO |
| **Ruby** | `.rb` | `ruby` | ✅ NUEVO |
| **Go** | `.go` | `go run` | ✅ NUEVO |
| **Rust** | `.rs` | `rustc` | ✅ NUEVO |
| **Bash** | `.sh` | `bash` | ✅ Funcionando |

## 🆕 Lenguajes Recién Agregados

### 1. Java ☕

**Código de ejemplo:**
```java
public class HelloWorld {
    public static void main(String[] args) {
        System.out.println("Hello from Java!");
        System.out.println("Robot Angel IDE");
    }
}
```

**Cómo funciona:**
1. Extrae el nombre de la clase del código (ej: `HelloWorld`)
2. Crea archivo con el nombre correcto: `HelloWorld.java`
3. Compila: `javac HelloWorld.java`
4. Ejecuta: `java HelloWorld`

**Requisitos:**
- Java JDK instalado
- `javac` y `java` en PATH

**Verificar instalación:**
```bash
javac -version
java -version
```

### 2. HTML 🌐

**Código de ejemplo:**
```html
<!DOCTYPE html>
<html>
<head>
    <title>Robot Angel Test</title>
</head>
<body>
    <h1>Hello from Robot Angel IDE!</h1>
    <p>This HTML was executed from the IDE.</p>
</body>
</html>
```

**Cómo funciona:**
1. Crea archivo temporal `.html`
2. Busca navegador disponible (`xdg-open`, `google-chrome`, `firefox`, `chromium-browser`)
3. Abre el archivo en el navegador

**Nota:**
- El HTML se abre en tu navegador predeterminado
- Si no encuentra navegador, muestra la ruta del archivo
- Puedes abrir el archivo manualmente

### 3. PHP 🐘

**Código de ejemplo:**
```php
<?php
echo "Hello from PHP!\n";
echo "Robot Angel IDE\n";
$nombre = "Usuario";
echo "Bienvenido, $nombre!\n";
?>
```

**Requisitos:**
- PHP instalado: `sudo apt install php`

### 4. Ruby 💎

**Código de ejemplo:**
```ruby
puts "Hello from Ruby!"
puts "Robot Angel IDE"
5.times do |i|
  puts "Contador: #{i}"
end
```

**Requisitos:**
- Ruby instalado: `sudo apt install ruby`

### 5. Go 🐹

**Código de ejemplo:**
```go
package main

import "fmt"

func main() {
    fmt.Println("Hello from Go!")
    fmt.Println("Robot Angel IDE")
}
```

**Requisitos:**
- Go instalado: `sudo apt install golang`

### 6. Rust 🦀

**Código de ejemplo:**
```rust
fn main() {
    println!("Hello from Rust!");
    println!("Robot Angel IDE");
}
```

**Requisitos:**
- Rust instalado: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

## 🚀 Cómo Usar

### Paso 1: Crear Archivo

En Robot Angel IDE, crea un archivo con la extensión apropiada:
- `test.java` para Java
- `test.html` para HTML
- `test.php` para PHP
- `test.rb` para Ruby
- `test.go` para Go
- `test.rs` para Rust

### Paso 2: Escribir Código

Escribe tu código en el editor Monaco.

### Paso 3: Ejecutar

Click en **Run** ▶️ y verás el output en la terminal.

## 📝 Ejemplos de Prueba

### Java - Operaciones Matemáticas
```java
public class Calculator {
    public static void main(String[] args) {
        int a = 10;
        int b = 5;
        System.out.println("Suma: " + (a + b));
        System.out.println("Resta: " + (a - b));
        System.out.println("Multiplicación: " + (a * b));
        System.out.println("División: " + (a / b));
    }
}
```

### HTML - Página Interactiva
```html
<!DOCTYPE html>
<html>
<head>
    <title>Robot Angel</title>
    <style>
        body { font-family: Arial; background: #1a1a1a; color: white; }
        h1 { color: #9d4edd; }
    </style>
</head>
<body>
    <h1>Robot Angel IDE</h1>
    <button onclick="alert('¡Hola desde el IDE!')">Click Me!</button>
</body>
</html>
```

### PHP - Variables y Loops
```php
<?php
$nombres = ["Alice", "Bob", "Charlie"];
foreach ($nombres as $nombre) {
    echo "Hola, $nombre!\n";
}

echo "\nCálculo: " . (10 * 5) . "\n";
?>
```

### Ruby - Arrays y Bloques
```ruby
nombres = ["Alice", "Bob", "Charlie"]
nombres.each do |nombre|
  puts "Hola, #{nombre}!"
end

(1..5).each { |i| puts "Número: #{i}" }
```

### Go - Goroutines (sin concurrencia para simplificar)
```go
package main

import "fmt"

func main() {
    for i := 0; i < 5; i++ {
        fmt.Printf("Iteración: %d\n", i)
    }
}
```

### Rust - Ownership Demo
```rust
fn main() {
    let nombre = String::from("Robot Angel");
    println!("IDE: {}", nombre);
    
    let numeros = vec![1, 2, 3, 4, 5];
    for n in &numeros {
        println!("Número: {}", n);
    }
}
```

## 🔧 Instalación de Dependencias

### Ubuntu/Debian

```bash
# Java
sudo apt install default-jdk

# PHP
sudo apt install php

# Ruby
sudo apt install ruby

# Go
sudo apt install golang

# Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Verificar Instalaciones

```bash
javac -version && java -version
php -v
ruby -v
go version
rustc --version
```

## ⚠️ Consideraciones Especiales

### Java
- El nombre de la clase DEBE coincidir con el nombre público
- Si tu código tiene `public class MiClase`, el sistema automáticamente crea `MiClase.java`
- Los archivos `.class` se generan en `/tmp` y se limpian automáticamente

### HTML
- Se abre en navegador externo, NO en terminal
- Si no detecta navegador, muestra la ruta del archivo
- Puedes abrir el archivo manualmente desde `/tmp/robot_angel_X.html`

### Lenguajes Compilados (C++, Java, Rust)
- Requieren paso de compilación adicional
- Timeout de 30 segundos para compilación
- Errores de compilación se muestran en la terminal

### Go
- Usa `go run` que compila y ejecuta en un solo paso
- No crea ejecutable permanente

## 📊 Detección Automática de Lenguaje

El sistema detecta automáticamente el lenguaje por extensión:

```typescript
.py    → Python
.js    → JavaScript
.ts    → TypeScript
.java  → Java
.html  → HTML
.php   → PHP
.rb    → Ruby
.go    → Go
.rs    → Rust
.cpp   → C++
.c     → C
.sh    → Bash
```

## 🐛 Troubleshooting

### "command not found: javac"
```bash
sudo apt install default-jdk
```

### "command not found: php"
```bash
sudo apt install php
```

### "No browser found" para HTML
El archivo se creó en `/tmp/robot_angel_X.html`. Ábrelo manualmente o instala un navegador:
```bash
sudo apt install firefox
```

### Errores de compilación Java
Asegúrate de que tu clase pública coincida con el nombre en el código:
```java
// CORRECTO
public class MiPrograma { ... }

// INCORRECTO (no funcionará si el nombre no coincide)
public class OtroNombre { ... }
```

## 🎉 Estado: ✅ FUNCIONANDO

Todos los lenguajes están implementados y listos para usar. Solo asegúrate de tener los compiladores/intérpretes instalados en tu sistema.

## 📚 Archivos Modificados

- `src/node/terminal-backend.ts` - Agregados casos para Java, HTML, PHP, Ruby, Go, Rust
- `src/App.tsx` - Detección de extensiones actualizada
- `MULTI_LANGUAGE_SUPPORT.md` - Esta documentación

---

¡Ahora puedes programar en 11 lenguajes diferentes desde Robot Angel IDE! 🚀✨
