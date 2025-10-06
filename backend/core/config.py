from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    APP_NAME: str = "Robot Angel"
    APP_VERSION: str = "0.1.0"
    API_PREFIX: str = "/api"
    DEBUG: bool = True

    class Config:
        env_file = ".env"

settings = Settings()
