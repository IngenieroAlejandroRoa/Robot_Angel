import { injectable } from '@theia/core/shared/inversify';
import { exec, spawn } from 'child_process';
import { promisify } from 'util';
import { TerminalCommandResult } from '../common/terminal-protocol';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';

const execAsync = promisify(exec);

export const TerminalBackendPath = '/services/angel-terminal-backend';

@injectable()
export class TerminalBackendImpl {
    private currentWorkingDirectory: string = process.cwd();
    private runningProcesses: Map<number, any> = new Map();
    private processCounter: number = 1;

    async executeCommand(command: string, cwd?: string): Promise<TerminalCommandResult> {
        const workDir = cwd || this.currentWorkingDirectory;
        
        try {
            const { stdout, stderr } = await execAsync(command, {
                cwd: workDir,
                timeout: 30000,
                maxBuffer: 1024 * 1024,
                shell: '/bin/bash'
            });

            if (command.trim().startsWith('cd ')) {
                const targetDir = command.trim().substring(3).trim();
                if (targetDir && !stderr) {
                    const { stdout: newCwd } = await execAsync('pwd', { cwd: targetDir, shell: '/bin/bash' });
                    this.currentWorkingDirectory = newCwd.trim();
                }
            }

            return {
                output: stdout,
                error: stderr,
                exitCode: 0
            };
        } catch (error: any) {
            return {
                output: error.stdout || '',
                error: error.stderr || error.message,
                exitCode: error.code || 1
            };
        }
    }

    async getWorkingDirectory(): Promise<string> {
        return this.currentWorkingDirectory;
    }

    async runScript(code: string, language: string): Promise<number> {
        const pid = this.processCounter++;
        
        try {
            // Create temp file
            const tempDir = os.tmpdir();
            let tempFile: string;
            let command: string;
            let args: string[];

            switch (language.toLowerCase()) {
                case 'python':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.py`);
                    fs.writeFileSync(tempFile, code);
                    command = 'python3';
                    args = [tempFile];
                    break;
                
                case 'javascript':
                case 'typescript':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.js`);
                    fs.writeFileSync(tempFile, code);
                    command = 'node';
                    args = [tempFile];
                    break;
                
                case 'cpp':
                case 'c++':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.cpp`);
                    const outputFile = path.join(tempDir, `robot_angel_${pid}`);
                    fs.writeFileSync(tempFile, code);
                    
                    // Compile first
                    await execAsync(`g++ "${tempFile}" -o "${outputFile}"`, {
                        cwd: this.currentWorkingDirectory,
                        timeout: 30000
                    });
                    
                    command = outputFile;
                    args = [];
                    break;
                
                default:
                    // Default to bash script
                    tempFile = path.join(tempDir, `robot_angel_${pid}.sh`);
                    fs.writeFileSync(tempFile, code);
                    fs.chmodSync(tempFile, '755');
                    command = 'bash';
                    args = [tempFile];
            }

            // Spawn process
            const proc = spawn(command, args, {
                cwd: this.currentWorkingDirectory,
                shell: true,
                detached: false
            });

            // Store process info
            this.runningProcesses.set(pid, {
                process: proc,
                tempFile,
                startTime: Date.now(),
                command,
                args
            });

            // Just log for now - output will be captured differently
            proc.stdout?.on('data', (data) => {
                console.log(`[PID ${pid}] stdout:`, data.toString());
            });

            proc.stderr?.on('data', (data) => {
                console.error(`[PID ${pid}] stderr:`, data.toString());
            });

            proc.on('close', (code) => {
                console.log(`[PID ${pid}] Process exited with code ${code}`);
                this.cleanupProcess(pid);
            });

            return pid;
        } catch (error) {
            console.error('Error running script:', error);
            throw error;
        }
    }

    async stopProcess(pid: number): Promise<boolean> {
        const procInfo = this.runningProcesses.get(pid);
        if (!procInfo) {
            return false;
        }

        try {
            procInfo.process.kill('SIGTERM');
            
            // Force kill after 2 seconds if still running
            setTimeout(() => {
                if (!procInfo.process.killed) {
                    procInfo.process.kill('SIGKILL');
                }
            }, 2000);

            this.cleanupProcess(pid);
            return true;
        } catch (error) {
            console.error(`Error stopping process ${pid}:`, error);
            return false;
        }
    }

    async isProcessRunning(pid: number): Promise<boolean> {
        const procInfo = this.runningProcesses.get(pid);
        return procInfo !== undefined && !procInfo.process.killed;
    }

    async getScriptCommand(code: string, language: string): Promise<string> {
        const pid = this.processCounter++;
        const tempDir = os.tmpdir();
        let tempFile: string;
        let command: string;

        try {
            switch (language.toLowerCase()) {
                case 'python':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.py`);
                    fs.writeFileSync(tempFile, code);
                    command = `python3 "${tempFile}"`;
                    break;
                
                case 'javascript':
                case 'typescript':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.js`);
                    fs.writeFileSync(tempFile, code);
                    command = `node "${tempFile}"`;
                    break;
                
                case 'cpp':
                case 'c++':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.cpp`);
                    const outputFile = path.join(tempDir, `robot_angel_${pid}`);
                    fs.writeFileSync(tempFile, code);
                    
                    // Compile first
                    await execAsync(`g++ "${tempFile}" -o "${outputFile}"`, {
                        cwd: this.currentWorkingDirectory,
                        timeout: 30000
                    });
                    
                    command = `"${outputFile}"`;
                    break;
                
                default:
                    // Default to bash script
                    tempFile = path.join(tempDir, `robot_angel_${pid}.sh`);
                    fs.writeFileSync(tempFile, code);
                    fs.chmodSync(tempFile, '755');
                    command = `bash "${tempFile}"`;
            }

            // Store file for cleanup
            this.runningProcesses.set(pid, {
                tempFile,
                command,
                startTime: Date.now()
            });

            return command;
        } catch (error) {
            console.error('Error creating script command:', error);
            throw error;
        }
    }

    private cleanupProcess(pid: number): void {
        const procInfo = this.runningProcesses.get(pid);
        if (procInfo) {
            // Delete temp file
            try {
                if (fs.existsSync(procInfo.tempFile)) {
                    fs.unlinkSync(procInfo.tempFile);
                }
            } catch (error) {
                console.error(`Error deleting temp file for PID ${pid}:`, error);
            }

            this.runningProcesses.delete(pid);
        }
    }
}
