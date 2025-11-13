#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Board Manager for detecting and managing ESP32 and Arduino boards
"""

import subprocess
import json
from typing import List, Dict, Optional
from pathlib import Path


class BoardInfo:
    def __init__(self, port: str, board_type: str, fqbn: str = ""):
        self.port = port
        self.board_type = board_type
        self.fqbn = fqbn or self._get_default_fqbn(board_type)
    
    def _get_default_fqbn(self, board_type: str) -> str:
        """Get default FQBN for board type"""
        fqbn_map = {
            "ESP32": "esp32:esp32:esp32",
            "ESP32-S2": "esp32:esp32:esp32s2",
            "ESP32-S3": "esp32:esp32:esp32s3",
            "ESP32-C3": "esp32:esp32:esp32c3",
            "Arduino Uno": "arduino:avr:uno",
            "Arduino Mega": "arduino:avr:mega",
            "Arduino Nano": "arduino:avr:nano",
        }
        return fqbn_map.get(board_type, "esp32:esp32:esp32")


class BoardManager:
    def __init__(self):
        self.arduino_cli = self._find_arduino_cli()
    
    def _find_arduino_cli(self) -> Optional[str]:
        """Find arduino-cli in PATH or common locations"""
        try:
            result = subprocess.run(
                ["which", "arduino-cli"],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                return result.stdout.strip()
        except:
            pass
        
        # Check common locations
        common_paths = [
            Path.home() / "tools" / "bin" / "arduino-cli",
            Path("/usr/local/bin/arduino-cli"),
            Path("/usr/bin/arduino-cli"),
        ]
        
        for path in common_paths:
            if path.exists():
                return str(path)
        
        return None
    
    def detect_boards(self) -> List[BoardInfo]:
        """Detect connected boards using arduino-cli"""
        if not self.arduino_cli:
            return []
        
        try:
            result = subprocess.run(
                [self.arduino_cli, "board", "list", "--format", "json"],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                return []
            
            data = json.loads(result.stdout)
            boards = []
            
            # Handle new format: {"detected_ports": [...]}
            detected_ports = data.get("detected_ports", [])
            if isinstance(detected_ports, list):
                ports_list = detected_ports
            else:
                # Fallback to old format
                ports_list = data if isinstance(data, list) else []
            
            for item in ports_list:
                port = item.get("address", "") or item.get("port", {}).get("address", "")
                matching_boards = item.get("matching_boards", [])
                
                if matching_boards:
                    board = matching_boards[0]
                    board_name = board.get("name", "Unknown")
                    fqbn = board.get("fqbn", "")
                    
                    # Determine board type
                    board_type = "ESP32"
                    if "ESP32" in board_name.upper():
                        if "S2" in board_name:
                            board_type = "ESP32-S2"
                        elif "S3" in board_name:
                            board_type = "ESP32-S3"
                        elif "C3" in board_name:
                            board_type = "ESP32-C3"
                    elif "Arduino" in board_name:
                        board_type = board_name
                    
                    boards.append(BoardInfo(port, board_type, fqbn))
                elif port:
                    # Unknown board but port detected
                    boards.append(BoardInfo(port, "ESP32", ""))
            
            return boards
        except Exception as e:
            print(f"Error detecting boards: {e}")
            return []
    
    def get_serial_ports(self) -> List[str]:
        """Get all serial ports"""
        try:
            result = subprocess.run(
                ["ls", "/dev/ttyUSB*", "/dev/ttyACM*"],
                capture_output=True,
                text=True,
                shell=False
            )
            ports = result.stdout.strip().split('\n')
            return [p for p in ports if p]
        except:
            return []
    
    def install_esp32_core(self) -> bool:
        """Install ESP32 core if not present"""
        if not self.arduino_cli:
            return False
        
        try:
            # Add ESP32 board manager URL
            subprocess.run(
                [
                    self.arduino_cli, "config", "add",
                    "board_manager.additional_urls",
                    "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
                ],
                capture_output=True
            )
            
            # Update index
            subprocess.run(
                [self.arduino_cli, "core", "update-index"],
                capture_output=True,
                timeout=60
            )
            
            # Install ESP32 core
            result = subprocess.run(
                [self.arduino_cli, "core", "install", "esp32:esp32"],
                capture_output=True,
                timeout=300
            )
            
            return result.returncode == 0
        except Exception as e:
            print(f"Error installing ESP32 core: {e}")
            return False
