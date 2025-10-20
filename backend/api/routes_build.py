from fastapi import APIRouter
from backend.core.utils import run_cli_command

router = APIRouter(prefix="/build", tags=["Build"])

@router.post("/")
def build_project():
    output = run_cli_command("robot-angel build")
    return {"status": "ok", "output": output}

@router.post("/flash")
def flash_device():
    output = run_cli_command("robot-angel flash")
    return {"status": "ok", "output": output}

