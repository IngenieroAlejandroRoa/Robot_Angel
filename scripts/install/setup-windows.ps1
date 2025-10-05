# ==============================================================
# Robot Angel - Instalador general para Windows
# ==============================================================

Write-Host "🤖 Instalando entorno Robot Angel en Windows..." -ForegroundColor Green

# --- Crear entorno virtual ---
$VenvPath = "$env:USERPROFILE\.venvs\robot-angel"
if (!(Test-Path $VenvPath)) {
    Write-Host "📦 Creando entorno virtual Python..." -ForegroundColor Yellow
    python -m venv $VenvPath
}
& "$VenvPath\Scripts\activate.ps1"

# --- Instalar dependencias Python ---
pip install --upgrade pip
pip install -r "$PSScriptRoot\requirements.txt"

# --- Instalar Arduino CLI ---
$ToolsDir = "$env:USERPROFILE\tools"
New-Item -ItemType Directory -Force -Path $ToolsDir | Out-Null
$ArduinoCLI = "$ToolsDir\arduino-cli.exe"
if (!(Test-Path $ArduinoCLI)) {
    Write-Host "🔩 Instalando Arduino CLI..." -ForegroundColor Yellow
    Invoke-WebRequest https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip -OutFile "$ToolsDir\arduino-cli.zip"
    Expand-Archive "$ToolsDir\arduino-cli.zip" -DestinationPath $ToolsDir -Force
    Remove-Item "$ToolsDir\arduino-cli.zip"
    [Environment]::SetEnvironmentVariable("PATH", "$env:PATH;$ToolsDir", [EnvironmentVariableTarget]::Machine)
}

# --- Instalar dependencias ROS 2 ---
Write-Host "⚙️  Verifica que tengas ROS 2 Jazzy instalado (solo si usas WSL o setup manual)." -ForegroundColor Yellow

# --- micro-ROS Agent (manual build) ---
$UrosPath = "$env:USERPROFILE\uros_ws"
if (!(Test-Path $UrosPath)) {
    Write-Host "🚀 Clonando micro-ROS Agent..." -ForegroundColor Green
    git clone https://github.com/micro-ROS/micro-ROS-Agent.git "$UrosPath\src\micro-ROS-Agent"
}
Write-Host "📦 Para compilar en Windows: usa WSL o Ubuntu nativo." -ForegroundColor Yellow

Write-Host "✅ Instalación completada. Ejecuta el IDE o activa el entorno:" -ForegroundColor Green
Write-Host "    & $VenvPath\Scripts\activate.ps1"
