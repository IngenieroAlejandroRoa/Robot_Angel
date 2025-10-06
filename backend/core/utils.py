import subprocess
from rich.console import Console

console = Console()

def run_cli_command(command: str) -> str:
    """Ejecuta un comando del CLI de Robot Angel y devuelve su salida."""
    console.print(f"⚙️ Ejecutando: {command}", style="bold cyan")
    try:
        result = subprocess.run(
            command, shell=True, capture_output=True, text=True, check=True
        )
        return result.stdout
    except subprocess.CalledProcessError as e:
        console.print(f"❌ Error ejecutando {command}", style="bold red")
        return e.stderr

