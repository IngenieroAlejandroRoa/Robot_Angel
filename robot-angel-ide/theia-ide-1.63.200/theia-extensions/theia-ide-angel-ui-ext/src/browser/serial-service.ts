import { injectable, inject } from '@theia/core/shared/inversify';
import { WebSocketConnectionProvider } from '@theia/core/lib/browser';

export interface SerialMessage {
    type: 'data' | 'error' | 'connect' | 'disconnect' | 'sent';
    content: string;
    timestamp: string;
}

export const SerialBackend = Symbol('SerialBackend');
export const SerialBackendPath = '/services/serial-backend';

export interface SerialBackend {
    connect(port: string, baudRate: number): Promise<{ success: boolean; message: string }>;
    disconnect(): Promise<{ success: boolean; message: string }>;
    sendData(data: string): Promise<{ success: boolean; message: string }>;
    isConnected(): Promise<boolean>;
    getCurrentConnection(): Promise<{ port: string; baudRate: number } | null>;
    getMessages(since: number): Promise<SerialMessage[]>;
    getMessageCount(): Promise<number>;
}

@injectable()
export class SerialService {
    private backend: SerialBackend;

    constructor(
        @inject(WebSocketConnectionProvider) protected readonly connectionProvider: WebSocketConnectionProvider
    ) {
        this.backend = this.connectionProvider.createProxy<SerialBackend>(SerialBackendPath);
    }

    async connect(port: string, baudRate: number): Promise<{ success: boolean; message: string }> {
        try {
            return await this.backend.connect(port, baudRate);
        } catch (error) {
            console.error('Failed to connect:', error);
            return { success: false, message: error instanceof Error ? error.message : 'Unknown error' };
        }
    }

    async disconnect(): Promise<{ success: boolean; message: string }> {
        try {
            return await this.backend.disconnect();
        } catch (error) {
            console.error('Failed to disconnect:', error);
            return { success: false, message: error instanceof Error ? error.message : 'Unknown error' };
        }
    }

    async sendData(data: string): Promise<{ success: boolean; message: string }> {
        try {
            return await this.backend.sendData(data);
        } catch (error) {
            console.error('Failed to send data:', error);
            return { success: false, message: error instanceof Error ? error.message : 'Unknown error' };
        }
    }

    async isConnected(): Promise<boolean> {
        try {
            return await this.backend.isConnected();
        } catch (error) {
            console.error('Failed to check connection:', error);
            return false;
        }
    }

    async getCurrentConnection(): Promise<{ port: string; baudRate: number } | null> {
        try {
            return await this.backend.getCurrentConnection();
        } catch (error) {
            console.error('Failed to get current connection:', error);
            return null;
        }
    }

    async getMessages(since: number = 0): Promise<SerialMessage[]> {
        try {
            return await this.backend.getMessages(since);
        } catch (error) {
            console.error('Failed to get messages:', error);
            return [];
        }
    }

    async getMessageCount(): Promise<number> {
        try {
            return await this.backend.getMessageCount();
        } catch (error) {
            console.error('Failed to get message count:', error);
            return 0;
        }
    }
}
