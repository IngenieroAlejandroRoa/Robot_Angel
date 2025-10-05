import os
import subprocess
import shutil
import click
from rich.console import Console

console = Console()

ROS_SETUP_CANDIDATES = [
    os.path.expanduser("~/uros_ws/install/setup.bash"),  # tu workspace micro-ROS
    "/opt/ros/jazzy/setup.bash",                         # ROS 2 Jazzy global
]


def _compose_ros_source_cmd() -> str | None:
    """Devuelve una cadena de 'source ... && source ...' con los setups que existan, o None si no hay ninguno."""
    sources = [p for p in ROS_SETUP_CANDIDATES if os.path.exists(p)]
    if not sources:
        return None
    return " && ".join([f"source {p}" for p in sources])


@click.command(help="🛰️ Inicia el micro-ROS Agent (por defecto UDP4:8888)")
@click.option(
    "--transport",
    type=click.Choice(["udp4", "udp6", "tcp4", "tcp6", "serial", "multiserial", "pseudoterminal", "canfd"]),
    default="udp4",
    show_default=True,
)
@click.option("--port", "-p", type=int, default=8888, show_default=True, help="Puerto para udp*/tcp*")
@click.option("--dev", "-D", default="", help="Dispositivo para serial/multiserial/canfd (e.g. /dev/ttyUSB0)")
@click.option("--baudrate", "-b", type=int, default=115200, show_default=True, help="Baudios para serial")
@click.option("--middleware", "-m", type=click.Choice(["ced", "dds", "rtps"]), default="dds", show_default=True)
@click.option("--discovery", "-d", type=int, default=7400, show_default=True)
@click.option("--verbose", "-v", is_flag=True, help="Salida verbosa del agente")
def run_agent(transport, port, dev, baudrate, middleware, discovery, verbose):
    # Verificar 'ros2'
    if not shutil.which("ros2"):
        console.print("❌ [red]Comando 'ros2' no disponible en PATH[/red]")
    src_cmd = _compose_ros_source_cmd()
    if not src_cmd:
        console.print("❌ [red]No encontré ningún setup de ROS 2/micro-ROS para 'source'.[/red]")
        console.print("   Prueba compilar el agente:  [bold]cd ~/uros_ws && colcon build --symlink-install[/bold]")
        return

    # Construir argumentos por transporte
    args = []
    args += ["-m", middleware]
    args += ["-d", str(discovery)]
    if verbose:
        args += ["-v"]

    if transport in ("udp4", "udp6", "tcp4", "tcp6"):
        args += ["-p", str(port)]
    elif transport in ("serial", "multiserial", "pseudoterminal", "canfd"):
        if not dev:
            console.print("❌ [red]Debes especificar --dev para transporte serial/multi/canfd[/red]")
            return
        args += ["-D", dev]
        if transport in ("serial", "multiserial", "pseudoterminal"):
            args += ["-b", str(baudrate)]

    # Ejecutar
    full = f"""{src_cmd} && ros2 run micro_ros_agent micro_ros_agent {transport} {' '.join(args)}"""
    console.print(f"🚀 Lanzando micro-ROS Agent: [dim]{transport} {' '.join(args)}[/dim]")
    # Nota: dejamos el proceso en primer plano (bloqueante). CTRL+C para cancelar.
    subprocess.run(["bash", "-lc", full], check=False)
