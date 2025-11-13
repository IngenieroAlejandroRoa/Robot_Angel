#!/usr/bin/env python3
"""
Serial Monitor Backend for Robot Angel IDE
Handles bidirectional serial communication
"""
import sys
import serial
import json
import threading
import time
import select


class SerialMonitor:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.running = False
        
    def send_message(self, msg_type, content='', **kwargs):
        """Send JSON message to stdout"""
        msg = {
            'type': msg_type,
            'content': content,
            **kwargs
        }
        print(json.dumps(msg), flush=True)
    
    def connect(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=1
            )
            self.send_message('connect', f'Connected to {self.port} at {self.baud_rate} baud', success=True)
            self.running = True
            return True
        except serial.SerialException as e:
            self.send_message('error', str(e), success=False)
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.send_message('disconnect', 'Disconnected')
    
    def read_loop(self):
        """Read data from serial port"""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        self.send_message('data', data)
            except serial.SerialException as e:
                self.send_message('error', f'Read error: {str(e)}')
                self.running = False
                break
            except Exception as e:
                self.send_message('error', f'Unexpected error: {str(e)}')
            time.sleep(0.01)
    
    def write_data(self, data):
        """Write data to serial port"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((data + '\n').encode('utf-8'))
                self.send_message('sent', data)
                return True
            else:
                self.send_message('error', 'Serial port not open')
                return False
        except serial.SerialException as e:
            self.send_message('error', f'Write error: {str(e)}')
            return False
    
    def input_loop(self):
        """Read commands from stdin"""
        while self.running:
            try:
                # Check if stdin has data available
                if sys.platform != 'win32':
                    # Unix-like systems
                    readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if readable:
                        line = sys.stdin.readline()
                        if line:
                            cmd = line.strip()
                            if cmd:
                                try:
                                    msg = json.loads(cmd)
                                    if msg.get('command') == 'send' and 'data' in msg:
                                        self.write_data(msg['data'])
                                    elif msg.get('command') == 'disconnect':
                                        self.running = False
                                        break
                                except json.JSONDecodeError:
                                    # Not JSON, treat as raw data to send
                                    self.write_data(cmd)
                else:
                    # Windows - simpler approach
                    time.sleep(0.1)
            except Exception as e:
                self.send_message('error', f'Input error: {str(e)}')
    
    def run(self):
        """Main run loop"""
        if not self.connect():
            return
        
        # Start read thread
        read_thread = threading.Thread(target=self.read_loop, daemon=True)
        read_thread.start()
        
        # Run input loop in main thread
        try:
            self.input_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.disconnect()
            read_thread.join(timeout=1)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(json.dumps({'type': 'error', 'content': 'Usage: serial_monitor.py <port> <baud_rate>'}), flush=True)
        sys.exit(1)
    
    port = sys.argv[1]
    try:
        baud_rate = int(sys.argv[2])
    except ValueError:
        print(json.dumps({'type': 'error', 'content': 'Invalid baud rate'}), flush=True)
        sys.exit(1)
    
    monitor = SerialMonitor(port, baud_rate)
    monitor.run()
