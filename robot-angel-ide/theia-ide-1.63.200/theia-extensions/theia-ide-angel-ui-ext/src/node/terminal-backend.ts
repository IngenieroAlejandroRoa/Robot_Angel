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
    private currentExecutionProcess: any = null;
    private microRosAgentProcess: any = null;

    async executeCommand(command: string, cwd?: string): Promise<TerminalCommandResult> {
        const workDir = cwd || this.currentWorkingDirectory;
        
        try {
            // Create a child process that we can kill
            const childProcess = exec(command, {
                cwd: workDir,
                timeout: 30000,
                maxBuffer: 1024 * 1024,
                shell: '/bin/bash'
            });

            // Store current process so we can kill it with Stop
            this.currentExecutionProcess = childProcess;

            // Wait for process to complete
            const result = await new Promise<TerminalCommandResult>((resolve, reject) => {
                let stdout = '';
                let stderr = '';

                childProcess.stdout?.on('data', (data) => {
                    stdout += data.toString();
                });

                childProcess.stderr?.on('data', (data) => {
                    stderr += data.toString();
                });

                childProcess.on('close', (code) => {
                    this.currentExecutionProcess = null;
                    resolve({
                        output: stdout,
                        error: stderr,
                        exitCode: code || 0
                    });
                });

                childProcess.on('error', (error) => {
                    this.currentExecutionProcess = null;
                    reject(error);
                });
            });

            if (command.trim().startsWith('cd ')) {
                const targetDir = command.trim().substring(3).trim();
                if (targetDir && !result.error) {
                    const { stdout: newCwd } = await execAsync('pwd', { cwd: targetDir, shell: '/bin/bash' });
                    this.currentWorkingDirectory = newCwd.trim();
                }
            }

            return result;
        } catch (error: any) {
            this.currentExecutionProcess = null;
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

    async sendInterruptSignal(): Promise<boolean> {
        if (this.currentExecutionProcess) {
            try {
                this.currentExecutionProcess.kill('SIGINT'); // Send Ctrl+C signal
                return true;
            } catch (error) {
                console.error('Error sending interrupt signal:', error);
                return false;
            }
        }
        return false;
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
                case 'js':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.js`);
                    fs.writeFileSync(tempFile, code);
                    command = `node "${tempFile}"`;
                    break;
                
                case 'cpp':
                case 'c++':
                case 'c':
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
                
                case 'java':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.java`);
                    fs.writeFileSync(tempFile, code);
                    
                    // Extract class name from code
                    const classMatch = code.match(/public\s+class\s+(\w+)/);
                    const className = classMatch ? classMatch[1] : `robot_angel_${pid}`;
                    
                    // If class name doesn't match filename, create proper file
                    if (className !== `robot_angel_${pid}`) {
                        tempFile = path.join(tempDir, `${className}.java`);
                        fs.writeFileSync(tempFile, code);
                    }
                    
                    // Compile Java
                    await execAsync(`javac "${tempFile}"`, {
                        cwd: tempDir,
                        timeout: 30000
                    });
                    
                    // Run Java directly from temp directory using absolute path
                    command = `java -cp "${tempDir}" ${className}`;
                    break;
                
                case 'html':
                case 'htm':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.html`);
                    fs.writeFileSync(tempFile, code);
                    
                    // Try different browsers in order of preference
                    const browsers = ['xdg-open', 'google-chrome', 'firefox', 'chromium-browser'];
                    let browserFound = false;
                    
                    for (const browser of browsers) {
                        try {
                            await execAsync(`which ${browser}`);
                            command = `${browser} "${tempFile}"`;
                            browserFound = true;
                            break;
                        } catch (e) {
                            // Browser not found, try next
                        }
                    }
                    
                    if (!browserFound) {
                        // If no browser found, just output the file path
                        command = `echo "HTML file created at: ${tempFile}" && echo "Open it manually in your browser"`;
                    }
                    break;
                
                case 'php':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.php`);
                    fs.writeFileSync(tempFile, code);
                    command = `php "${tempFile}"`;
                    break;
                
                case 'ruby':
                case 'rb':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.rb`);
                    fs.writeFileSync(tempFile, code);
                    command = `ruby "${tempFile}"`;
                    break;
                
                case 'go':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.go`);
                    fs.writeFileSync(tempFile, code);
                    command = `go run "${tempFile}"`;
                    break;
                
                case 'rust':
                case 'rs':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.rs`);
                    const rustOutput = path.join(tempDir, `robot_angel_${pid}_rust`);
                    fs.writeFileSync(tempFile, code);
                    
                    // Compile Rust
                    await execAsync(`rustc "${tempFile}" -o "${rustOutput}"`, {
                        cwd: this.currentWorkingDirectory,
                        timeout: 30000
                    });
                    
                    command = `"${rustOutput}"`;
                    break;
                
                case 'sh':
                case 'bash':
                    tempFile = path.join(tempDir, `robot_angel_${pid}.sh`);
                    fs.writeFileSync(tempFile, code);
                    fs.chmodSync(tempFile, '755');
                    command = `bash "${tempFile}"`;
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

    // ========== Micro-ROS Agent Methods ==========

    private findRosSetup(): string | null {
        const candidates = [
            path.join(os.homedir(), 'uros_ws/install/setup.bash'),
            '/opt/ros/jazzy/setup.bash',
            '/opt/ros/humble/setup.bash',
            '/opt/ros/iron/setup.bash',
        ];

        for (const candidate of candidates) {
            if (fs.existsSync(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    async startMicroRosAgent(config: any): Promise<boolean> {
        if (this.microRosAgentProcess) {
            console.log('Micro-ROS agent is already running');
            return false;
        }

        // Find ROS setup
        const rosSetup = this.findRosSetup();
        if (!rosSetup) {
            console.error('No ROS 2 setup found. Install ROS 2 or micro-ROS workspace.');
            return false;
        }

        // Build command arguments
        const args: string[] = [];
        args.push('-m', config.middleware || 'dds');
        args.push('-d', String(config.discovery || 7400));
        
        if (config.verbose) {
            args.push('-v');
        }

        const transport = config.transport || 'udp4';

        if (['udp4', 'udp6', 'tcp4', 'tcp6'].includes(transport)) {
            args.push('-p', String(config.port || 8888));
        } else if (['serial', 'multiserial', 'pseudoterminal', 'canfd'].includes(transport)) {
            if (!config.device) {
                console.error('Device required for serial/multi/canfd transport');
                return false;
            }
            args.push('-D', config.device);
            if (['serial', 'multiserial', 'pseudoterminal'].includes(transport)) {
                args.push('-b', String(config.baudrate || 115200));
            }
        }

        // Build full command
        const agentCmd = `ros2 run micro_ros_agent micro_ros_agent ${transport} ${args.join(' ')}`;
        const fullCmd = `source ${rosSetup} && ${agentCmd}`;

        console.log(`Starting micro-ROS agent: ${agentCmd}`);

        try {
            this.microRosAgentProcess = exec(fullCmd, {
                shell: '/bin/bash',
                maxBuffer: 1024 * 1024
            });

            this.microRosAgentProcess.stdout?.on('data', (data: any) => {
                console.log(`[micro-ROS agent] ${data.toString()}`);
            });

            this.microRosAgentProcess.stderr?.on('data', (data: any) => {
                console.error(`[micro-ROS agent ERROR] ${data.toString()}`);
            });

            this.microRosAgentProcess.on('close', (code: number) => {
                console.log(`Micro-ROS agent exited with code ${code}`);
                this.microRosAgentProcess = null;
            });

            this.microRosAgentProcess.on('error', (error: any) => {
                console.error('Micro-ROS agent error:', error);
                this.microRosAgentProcess = null;
            });

            return true;
        } catch (error) {
            console.error('Error starting micro-ROS agent:', error);
            this.microRosAgentProcess = null;
            return false;
        }
    }

    async stopMicroRosAgent(): Promise<boolean> {
        if (!this.microRosAgentProcess) {
            console.log('No micro-ROS agent running');
            return false;
        }

        try {
            this.microRosAgentProcess.kill('SIGINT');
            this.microRosAgentProcess = null;
            console.log('Micro-ROS agent stopped');
            return true;
        } catch (error) {
            console.error('Error stopping micro-ROS agent:', error);
            return false;
        }
    }

    async isMicroRosAgentRunning(): Promise<boolean> {
        return this.microRosAgentProcess !== null;
    }
}
