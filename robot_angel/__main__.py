from robot_angel.cli.core import cli  # grupo principal con build/flash/monitor
from robot_angel.cli.agent import run_agent
from robot_angel.cli.project import init_project

# enganchar subcomandos adicionales
cli.add_command(run_agent, name="run-agent")
cli.add_command(init_project, name="init-project")

if __name__ == "__main__":
    cli()
