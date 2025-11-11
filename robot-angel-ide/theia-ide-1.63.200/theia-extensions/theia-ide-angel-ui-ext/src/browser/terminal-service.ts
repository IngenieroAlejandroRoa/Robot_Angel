import { inject, injectable, postConstruct } from '@theia/core/shared/inversify';
import { TerminalService } from '@theia/terminal/lib/browser/base/terminal-service';
import { TerminalWidget } from '@theia/terminal/lib/browser/base/terminal-widget';

export interface TerminalHistoryEntry {
    type: 'system' | 'input' | 'output' | 'error' | 'success';
    content: string;
}

@injectable()
export class AngelTerminalService {
    @inject(TerminalService)
    protected readonly terminalService: TerminalService;

    private currentTerminal: TerminalWidget | undefined;
    private historyCallbacks: Set<(entry: TerminalHistoryEntry) => void> = new Set();
    private outputBuffer: string = '';

    @postConstruct()
    protected init(): void {
        console.log('AngelTerminalService initialized');
    }

    /**
     * Get or create the current terminal instance
     */
    async ensureTerminal(): Promise<TerminalWidget> {
        if (this.currentTerminal && !this.currentTerminal.isDisposed) {
            return this.currentTerminal;
        }

        this.currentTerminal = await this.terminalService.newTerminal({
            title: 'Robot Angel Terminal',
            destroyTermOnClose: false
        });

        // Listen to terminal output
        await this.setupTerminalListeners(this.currentTerminal);

        return this.currentTerminal;
    }

    /**
     * Execute a command in the terminal
     */
    async executeCommand(command: string): Promise<void> {
        const terminal = await this.ensureTerminal();
        
        console.log('Executing command:', command);
        
        // Notify listeners about the input
        this.notifyHistory({
            type: 'input',
            content: command
        });

        // Clear the output buffer before sending command
        this.outputBuffer = '';

        // Send the command with a newline to execute it
        // Use \r to simulate Enter key press
        terminal.sendText(command + '\r');
    }

    /**
     * Setup listeners for terminal output
     */
    private async setupTerminalListeners(terminal: TerminalWidget): Promise<void> {
        // Wait for the terminal to be fully initialized
        await new Promise(resolve => setTimeout(resolve, 500));

        const terminalImpl = terminal as any;
        
        console.log('Setting up terminal listeners, terminalId:', terminal.terminalId);
        console.log('Terminal object:', terminalImpl);

        // Try to access the xterm.js instance
        if (terminalImpl.term) {
            console.log('Found xterm instance');
            
            // Listen to data events from xterm
            terminalImpl.term.onData((data: string) => {
                console.log('Terminal data received:', data);
                this.processOutput(data);
            });

            // Also listen to the buffer
            const buffer = terminalImpl.term.buffer;
            if (buffer) {
                console.log('Terminal buffer available');
            }

            // Send initial prompt info
            this.notifyHistory({
                type: 'system',
                content: 'Terminal connected and ready'
            });
        } else {
            console.warn('Could not access xterm instance');
            this.notifyHistory({
                type: 'error',
                content: 'Warning: Terminal output capture may be limited'
            });
        }
    }

    /**
     * Process terminal output and extract meaningful information
     */
    private processOutput(data: string): void {
        // Accumulate output
        this.outputBuffer += data;

        // Check if we have a complete line (ends with newline)
        if (data.includes('\n') || data.includes('\r')) {
            const lines = this.outputBuffer.split(/\r?\n/);
            
            // Process all complete lines
            for (let i = 0; i < lines.length - 1; i++) {
                const line = lines[i].trim();
                if (line && !this.isPromptLine(line)) {
                    this.notifyHistory({
                        type: 'output',
                        content: line
                    });
                }
            }
            
            // Keep the last incomplete line in buffer
            this.outputBuffer = lines[lines.length - 1];
        }
    }

    /**
     * Check if a line is just a prompt (to avoid duplicating prompts)
     */
    private isPromptLine(line: string): boolean {
        // Common prompt patterns
        const promptPatterns = [
            /^[\w@-]+[:#\$>]\s*$/,  // user@host:~$
            /^\(.*\)\s*[\w@-]+[:#\$>]\s*$/,  // (venv) user@host:~$
        ];
        
        return promptPatterns.some(pattern => pattern.test(line));
    }

    /**
     * Register a callback for history updates
     */
    onHistoryUpdate(callback: (entry: TerminalHistoryEntry) => void): () => void {
        this.historyCallbacks.add(callback);
        return () => {
            this.historyCallbacks.delete(callback);
        };
    }

    /**
     * Notify all registered callbacks about a new history entry
     */
    private notifyHistory(entry: TerminalHistoryEntry): void {
        this.historyCallbacks.forEach(callback => {
            try {
                callback(entry);
            } catch (error) {
                console.error('Error in history callback:', error);
            }
        });
    }

    /**
     * Show the terminal widget
     */
    async showTerminal(): Promise<void> {
        const terminal = await this.ensureTerminal();
        await this.terminalService.open(terminal);
    }

    /**
     * Get the current terminal widget
     */
    getCurrentTerminal(): TerminalWidget | undefined {
        return this.currentTerminal;
    }

    /**
     * Get current working directory from terminal
     */
    async getCurrentDirectory(): Promise<string | undefined> {
        const terminal = this.currentTerminal;
        if (!terminal) return undefined;

        const terminalImpl = terminal as any;
        if (terminalImpl.cwd) {
            return terminalImpl.cwd.toString();
        }
        
        return undefined;
    }

    /**
     * Enable/disable terminal output echo for debugging
     */
    setDebugMode(enabled: boolean): void {
        console.log('Debug mode:', enabled);
    }
}
