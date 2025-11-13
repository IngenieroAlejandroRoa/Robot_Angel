#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arduino/ESP32 code uploader
"""

import subprocess
import tempfile
import os
from pathlib import Path
from typing import Optional, Callable


class ArduinoUploader:
    def __init__(self, arduino_cli_path: str):
        self.arduino_cli = arduino_cli_path
    
    def compile_and_upload(
        self,
        code: str,
        port: str,
        fqbn: str = "esp32:esp32:esp32",
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> tuple[bool, str]:
        """
        Compile and upload Arduino/ESP32 code
        
        Args:
            code: Arduino code to upload
            port: Serial port (e.g., /dev/ttyUSB0)
            fqbn: Fully Qualified Board Name
            progress_callback: Function to call with progress messages
        
        Returns:
            (success: bool, message: str)
        """
        
        def log(msg: str):
            if progress_callback:
                progress_callback(msg)
            print(msg)
        
        # Create temporary directory for sketch
        with tempfile.TemporaryDirectory() as tmpdir:
            sketch_dir = Path(tmpdir) / "sketch"
            sketch_dir.mkdir()
            
            # Write code to .ino file
            sketch_file = sketch_dir / "sketch.ino"
            sketch_file.write_text(code, encoding="utf-8")
            
            log(f"📝 Sketch created at {sketch_file}")
            
            # Compile
            log("🔨 Compiling...")
            compile_result = subprocess.run(
                [
                    self.arduino_cli, "compile",
                    "--fqbn", fqbn,
                    str(sketch_dir)
                ],
                capture_output=True,
                text=True,
                timeout=120
            )
            
            if compile_result.returncode != 0:
                error_msg = compile_result.stderr or compile_result.stdout
                log(f"❌ Compilation failed:\n{error_msg}")
                return False, f"Compilation error:\n{error_msg}"
            
            log("✅ Compilation successful")
            
            # Upload
            log(f"📤 Uploading to {port}...")
            upload_result = subprocess.run(
                [
                    self.arduino_cli, "upload",
                    "--fqbn", fqbn,
                    "--port", port,
                    str(sketch_dir)
                ],
                capture_output=True,
                text=True,
                timeout=60
            )
            
            if upload_result.returncode != 0:
                error_msg = upload_result.stderr or upload_result.stdout
                log(f"❌ Upload failed:\n{error_msg}")
                return False, f"Upload error:\n{error_msg}"
            
            log("✅ Upload successful!")
            return True, "Code uploaded successfully"
    
    def verify_only(
        self,
        code: str,
        fqbn: str = "esp32:esp32:esp32",
        progress_callback: Optional[Callable[[str], None]] = None
    ) -> tuple[bool, str]:
        """Verify/compile code without uploading"""
        
        def log(msg: str):
            if progress_callback:
                progress_callback(msg)
            print(msg)
        
        with tempfile.TemporaryDirectory() as tmpdir:
            sketch_dir = Path(tmpdir) / "sketch"
            sketch_dir.mkdir()
            
            sketch_file = sketch_dir / "sketch.ino"
            sketch_file.write_text(code, encoding="utf-8")
            
            log("🔨 Verifying code...")
            result = subprocess.run(
                [
                    self.arduino_cli, "compile",
                    "--fqbn", fqbn,
                    str(sketch_dir)
                ],
                capture_output=True,
                text=True,
                timeout=120
            )
            
            if result.returncode != 0:
                error_msg = result.stderr or result.stdout
                log(f"❌ Verification failed:\n{error_msg}")
                return False, error_msg
            
            log("✅ Verification successful")
            return True, "Code verified successfully"
