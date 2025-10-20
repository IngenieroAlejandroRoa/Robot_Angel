from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from backend.api import routes_projects, routes_build, routes_devices
from backend.core.config import settings

app = FastAPI(
    title="Robot Angel API",
    version="0.1.0",
    description="Backend de integración para el IDE Robot Angel (CLI + GUI).",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(routes_projects.router)
app.include_router(routes_build.router)
app.include_router(routes_devices.router)

@app.get("/")
async def root():
    return {"status": "🧠 Robot Angel API activa", "version": settings.APP_VERSION}

