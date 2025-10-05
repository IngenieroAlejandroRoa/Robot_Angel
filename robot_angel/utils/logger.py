from rich.console import Console

console = Console()

def log(message: str):
    console.print(f"[bold cyan]Robot Angel:[/bold cyan] {message}")
