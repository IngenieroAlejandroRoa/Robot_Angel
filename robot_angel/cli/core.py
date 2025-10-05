import os
import subprocess
import click
from rich.console import Console

console = Console()

ESP_IDF_PATH = os.path.expanduser("~/tools/esp-idf")
# Ajusta si usas otra versión de Python/IDF
ESP_PYTHON_ENV = os.path.expanduser("~/.espressif/python_env/idf5.3_py3.12_env/bin/python")


def _source_env_and_capture(command: str) -> dict:
    """Ejecuta `command` en bash y devuelve el entorno resultante como dict."""
    proc = subprocess.run(
        f"bash -c '{command} >/dev/null 2>&1; env'",
        shell=True,
        executable="/bin/bash",
        text=True,
        capture_output=True,
    )
    env = os.environ.copy()
    if proc.returncode == 0 and proc.stdout:
        for line in proc.stdout.splitlines():
            k, _, v = line.partition("=")
            if k:
                env[k] = v
    return env


def ensure_esp_idf_env() -> dict | None:
    """Verifica y exporta correctamente el entorno de ESP-IDF. Devuelve un env dict listo para usar con idf.py."""
    console.print("🔧 [bold cyan]Verificando entorno ESP-IDF...[/bold cyan]")

    export_script = os.path.join(ESP_IDF_PATH, "export.sh")
    if not os.path.exists(export_script):
        console.print("❌ [red]ESP-IDF no encontrado en ~/tools/esp-idf[/red]")
        return None

    # Crear entorno Python interno si no existe
    if not os.path.exists(ESP_PYTHON_ENV):
        console.print("⚙️  Creando entorno Python interno de ESP-IDF...")
        subprocess.run(
            f"bash -lc 'cd {ESP_IDF_PATH} && source export.sh && python3 tools/idf_tools.py install-python-env'",
            shell=True,
            executable="/bin/bash",
            check=False,
        )

    # Cargar variables de entorno del export.sh y devolverlas como dict
    env = _source_env_and_capture(f"source {export_script}")
    try:
        subprocess.run("bash -lc 'idf.py --version'", shell=True, executable="/bin/bash", check=True, env=env)
        console.print("✅ [green]ESP-IDF operativo[/green]")
        return env
    except subprocess.CalledProcessError:
        console.print("⚠️ [yellow]ESP-IDF detectado pero idf.py no respondió[/yellow]")
        return env  # igual devolvemos env para permitir reintentos


def run_idf_command(cmd: str):
    """Ejecuta un comando idf.py dentro del entorno exportado."""
    env = ensure_esp_idf_env()
    if not env:
        return
    subprocess.run(f"bash -lc 'idf.py {cmd}'", shell=True, executable="/bin/bash", cwd=os.getcwd(), env=env)


@click.group(help="🤖 Robot Angel - Open Source Robotics IDE CLI")
def cli():
    pass


@cli.command(help="🧱 Compila el proyecto actual (ESP-IDF o ROS2)")
def build():
    console.print("Robot Angel: Compilando proyecto...")
    run_idf_command("build")


@cli.command(help="⚡ Flashea el dispositivo conectado")
def flash():
    console.print("Robot Angel: Flasheando firmware...")
    run_idf_command("flash")


@cli.command(help="📡 Abre el monitor serial")
def monitor():
    console.print("Robot Angel: Abriendo monitor serial...")
    run_idf_command("monitor")
