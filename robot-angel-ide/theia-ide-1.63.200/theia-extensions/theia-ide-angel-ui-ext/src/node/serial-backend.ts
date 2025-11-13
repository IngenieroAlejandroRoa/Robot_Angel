import { injectable, inject } from '@theia/core/shared/inversify';
import { spawn, ChildProcess } from 'child_process';
import * as path from 'path';
import * as os from 'os';
import * as fs from 'fs';

export interface SerialMessage {
    type: 'data' | 'error' | 'connect' | 'disconnect' | 'sent';
    content: string;
    timestamp: string;
}

export const SerialBackendPath = '/services/serial-backend';

@injectable()
export class SerialBackendImpl {
    private pythonPath: string;
    private scriptPath: string;
    private serialProcess: ChildProcess | null = null;
    private currentPort: string = '';
    private currentBaudRate: number = 9600;
    private messageBuffer: SerialMessage[] = [];
    private maxBufferSize: number = 1000;

    constructor() {
        const venvPath = process.env.VIRTUAL_ENV || path.join(os.homedir(), '.venvs', 'robot-angel');
        this.pythonPath = path.join(venvPath, 'bin', 'python');
        this.scriptPath = path.join(os.homedir(), 'Desktop', 'RobotAngel', 'robot_angel');
    }

    async connect(port: string, baudRate: number): Promise<{ success: boolean; message: string }> {
        if (this.serialProcess) {
            return { success: false, message: 'Already connected. Disconnect first.' };
        }

        this.currentPort = port;
        this.currentBaudRate = baudRate;
        this.messageBuffer = []; // Clear buffer on new connection

        return new Promise((resolve) => {
            const serialScriptPath = path.join(this.scriptPath, 'utils', 'serial_monitor.py');
            
            this.serialProcess = spawn(this.pythonPath, [serialScriptPath, port, String(baudRate)]);
            
            let connected = false;

            this.serialProcess.stdout?.on('data', (data) => {
                const lines = data.toString().split('\n').filter((l: string) => l.trim());
                
                for (const line of lines) {
                    try {
                        const msg = JSON.parse(line);
                        
                        if (msg.type === 'connect' && !connected) {
                            connected = true;
                            resolve({ success: msg.success, message: msg.message });
                        }
                        
                        // Buffer the message
                        this.addMessage({
                            type: msg.type,
                            content: msg.content || msg.message || '',
                            timestamp: new Date().toLocaleTimeString()
                        });
                    } catch (e) {
                        // Not JSON, treat as raw data
                        this.addMessage({
                            type: 'data',
                            content: line,
                            timestamp: new Date().toLocaleTimeString()
                        });
                    }
                }
            });

            this.serialProcess.stderr?.on('data', (data) => {
                const error = data.toString();
                console.error('Serial error:', error);
                
                if (!connected) {
                    connected = true;
                    resolve({ success: false, message: error });
                }
                
                this.addMessage({
                    type: 'error',
                    content: error,
                    timestamp: new Date().toLocaleTimeString()
                });
            });

            this.serialProcess.on('close', (code) => {
                console.log(`Serial process exited with code ${code}`);
                this.serialProcess = null;
                
                this.addMessage({
                    type: 'disconnect',
                    content: 'Connection closed',
                    timestamp: new Date().toLocaleTimeString()
                });
            });

            this.serialProcess.on('error', (err) => {
                console.error('Serial process error:', err);
                this.serialProcess = null;
                
                if (!connected) {
                    connected = true;
                    resolve({ success: false, message: err.message });
                }
                
                this.addMessage({
                    type: 'error',
                    content: err.message,
                    timestamp: new Date().toLocaleTimeString()
                });
            });

            // Timeout if connection takes too long
            setTimeout(() => {
                if (!connected) {
                    connected = true;
                    this.disconnect();
                    resolve({ success: false, message: 'Connection timeout' });
                }
            }, 5000);
        });
    }

    private addMessage(message: SerialMessage): void {
        this.messageBuffer.push(message);
        // Keep buffer size under control
        if (this.messageBuffer.length > this.maxBufferSize) {
            this.messageBuffer.shift();
        }
    }

    async disconnect(): Promise<{ success: boolean; message: string }> {
        if (!this.serialProcess) {
            return { success: false, message: 'Not connected' };
        }

        return new Promise((resolve) => {
            if (this.serialProcess) {
                this.serialProcess.kill('SIGINT');
                
                setTimeout(() => {
                    if (this.serialProcess && !this.serialProcess.killed) {
                        this.serialProcess.kill('SIGKILL');
                    }
                    this.serialProcess = null;
                    resolve({ success: true, message: 'Disconnected' });
                }, 500);
            } else {
                resolve({ success: true, message: 'Already disconnected' });
            }
        });
    }

    async sendData(data: string): Promise<{ success: boolean; message: string }> {
        if (!this.serialProcess) {
            return { success: false, message: 'Not connected' };
        }

        try {
            const command = JSON.stringify({ command: 'send', data }) + '\n';
            this.serialProcess.stdin?.write(command);
            
            // Add to buffer
            this.addMessage({
                type: 'sent',
                content: data,
                timestamp: new Date().toLocaleTimeString()
            });
            
            return { success: true, message: 'Data sent' };
        } catch (error: any) {
            return { success: false, message: error.message };
        }
    }

    async isConnected(): Promise<boolean> {
        return this.serialProcess !== null && !this.serialProcess.killed;
    }

    async getMessages(since: number = 0): Promise<SerialMessage[]> {
        // Return messages from index 'since' onwards
        return this.messageBuffer.slice(since);
    }

    async getMessageCount(): Promise<number> {
        return this.messageBuffer.length;
    }

    async getCurrentConnection(): Promise<{ port: string; baudRate: number } | null> {
        if (this.serialProcess) {
            return {
                port: this.currentPort,
                baudRate: this.currentBaudRate
            };
        }
        return null;
    }
}
