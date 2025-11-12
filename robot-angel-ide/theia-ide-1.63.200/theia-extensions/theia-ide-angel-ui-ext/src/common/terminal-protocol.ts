export const TerminalBackendPath = '/services/angel-terminal-backend';
export const TerminalBackend = Symbol('TerminalBackend');

export interface TerminalCommandResult {
    output: string;
    error: string;
    exitCode: number;
}

export interface TerminalBackend {
    executeCommand(command: string, cwd?: string): Promise<TerminalCommandResult>;
    getWorkingDirectory(): Promise<string>;
}
