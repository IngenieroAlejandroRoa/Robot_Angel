export const TerminalBackendPath = '/services/angel-terminal-backend';
export const TerminalBackend = Symbol('TerminalBackend');

export interface TerminalCommandResult {
    output: string;
    error: string;
    exitCode: number;
}

export interface ProcessOutputEvent {
    pid: number;
    type: 'stdout' | 'stderr' | 'exit';
    data: string;
    exitCode?: number;
}

export interface TerminalBackend {
    executeCommand(command: string, cwd?: string): Promise<TerminalCommandResult>;
    getWorkingDirectory(): Promise<string>;
    runScript(code: string, language: string): Promise<number>;
    stopProcess(pid: number): Promise<boolean>;
    isProcessRunning(pid: number): Promise<boolean>;
    getScriptCommand(code: string, language: string): Promise<string>;
    sendInterruptSignal(): Promise<boolean>;
    startMicroRosAgent(config: MicroRosAgentConfig): Promise<boolean>;
    stopMicroRosAgent(): Promise<boolean>;
    isMicroRosAgentRunning(): Promise<boolean>;
    findRosSetup(): Promise<string | null>;
}

export interface MicroRosAgentConfig {
    transport: 'udp4' | 'udp6' | 'tcp4' | 'tcp6' | 'serial' | 'multiserial' | 'pseudoterminal' | 'canfd';
    port?: number;
    device?: string;
    baudrate?: number;
    middleware?: 'ced' | 'dds' | 'rtps';
    discovery?: number;
    verbose?: boolean;
}
