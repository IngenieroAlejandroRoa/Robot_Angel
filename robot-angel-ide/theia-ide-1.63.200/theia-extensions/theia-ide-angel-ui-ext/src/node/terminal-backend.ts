import { injectable } from '@theia/core/shared/inversify';
import { exec } from 'child_process';
import { promisify } from 'util';
import { TerminalCommandResult } from '../common/terminal-protocol';

const execAsync = promisify(exec);

export const TerminalBackendPath = '/services/angel-terminal-backend';

@injectable()
export class TerminalBackendImpl {
    private currentWorkingDirectory: string = process.cwd();

    async executeCommand(command: string, cwd?: string): Promise<TerminalCommandResult> {
        const workDir = cwd || this.currentWorkingDirectory;
        
        try {
            // Execute command with timeout
            const { stdout, stderr } = await execAsync(command, {
                cwd: workDir,
                timeout: 30000, // 30 seconds timeout
                maxBuffer: 1024 * 1024, // 1MB buffer
                shell: '/bin/bash'
            });

            // Update working directory if cd command
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
}
