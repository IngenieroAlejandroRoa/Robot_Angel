import { useEffect, useState, useCallback } from 'react';
import { AngelTerminalService, TerminalHistoryEntry } from '../browser/terminal-service';

// Singleton instance to share across components
let terminalServiceInstance: AngelTerminalService | null = null;

export function setTerminalServiceInstance(service: AngelTerminalService) {
    terminalServiceInstance = service;
}

export function useTerminalService() {
    const [history, setHistory] = useState<TerminalHistoryEntry[]>([
        { type: "system", content: "RoboIDE Terminal v1.0.0" },
        { type: "system", content: "Initializing terminal connection..." },
    ]);
    const [currentDir, setCurrentDir] = useState<string>('');

    useEffect(() => {
        if (!terminalServiceInstance) {
            console.warn('Terminal service not available yet');
            return;
        }

        console.log('Terminal service available, setting up...');

        // Subscribe to terminal updates
        const unsubscribe = terminalServiceInstance.onHistoryUpdate((entry) => {
            console.log('Received history entry:', entry);
            setHistory(prev => [...prev, entry]);
        });

        // Ensure terminal is created
        terminalServiceInstance.ensureTerminal().then(async () => {
            console.log('Terminal ensured successfully');
            
            // Get current directory
            const cwd = await terminalServiceInstance.getCurrentDirectory();
            if (cwd) {
                setCurrentDir(cwd);
                setHistory(prev => [
                    ...prev,
                    { type: 'system', content: `Working directory: ${cwd}` }
                ]);
            }
            
            // Send pwd to show current directory
            setTimeout(() => {
                terminalServiceInstance.executeCommand('pwd');
            }, 1000);
            
        }).catch(err => {
            console.error('Failed to ensure terminal:', err);
            setHistory(prev => [
                ...prev,
                { type: 'error', content: 'Failed to initialize terminal' }
            ]);
        });

        return unsubscribe;
    }, []);

    const executeCommand = useCallback(async (command: string) => {
        if (!terminalServiceInstance) {
            console.error('Terminal service not available');
            setHistory(prev => [
                ...prev,
                { type: 'error', content: 'Terminal service not available' }
            ]);
            return;
        }

        try {
            await terminalServiceInstance.executeCommand(command);
        } catch (error) {
            console.error('Failed to execute command:', error);
            setHistory(prev => [
                ...prev,
                { type: 'error', content: `Error: ${error}` }
            ]);
        }
    }, []);

    const showTerminal = useCallback(async () => {
        if (!terminalServiceInstance) {
            console.error('Terminal service not available');
            return;
        }

        try {
            await terminalServiceInstance.showTerminal();
        } catch (error) {
            console.error('Failed to show terminal:', error);
        }
    }, []);

    return {
        history,
        executeCommand,
        showTerminal,
        currentDir,
        isAvailable: terminalServiceInstance !== null
    };
}
