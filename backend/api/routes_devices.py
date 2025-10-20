from fastapi import APIRouter
import serial.tools.list_ports

router = APIRouter(prefix="/devices", tags=["Devices"])

@router.get("/")
def list_connected_devices():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return {"connected_devices": ports}

