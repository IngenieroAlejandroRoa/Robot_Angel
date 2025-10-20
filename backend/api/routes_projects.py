from fastapi import APIRouter
from backend.core.utils import run_cli_command

router = APIRouter(prefix="/projects", tags=["Projects"])

@router.post("/init")
def init_project(name: str = "demo"):
    output = run_cli_command(f"robot-angel init-project {name}")
    return {"message": f"Proyecto {name} creado.", "output": output}

@router.get("/list")
def list_projects():
    return {"projects": ["demo", "esp32-arm", "ros-agent"]}

