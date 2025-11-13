import { injectable, inject } from '@theia/core/shared/inversify';
import { WebSocketConnectionProvider } from '@theia/core/lib/browser';

export interface BoardInfo {
    port: string;
    boardType: string;
    fqbn: string;
}

export interface UploadProgress {
    stage: 'compiling' | 'uploading' | 'done' | 'error';
    message: string;
}

export const BoardManagerBackend = Symbol('BoardManagerBackend');
export const BoardManagerBackendPath = '/services/board-manager';

export interface BoardManagerBackend {
    detectBoards(): Promise<BoardInfo[]>;
    getSerialPorts(): Promise<string[]>;
    uploadCode(code: string, port: string, fqbn: string): Promise<{ success: boolean; message: string }>;
    verifyCode(code: string, fqbn: string): Promise<{ success: boolean; message: string }>;
}

@injectable()
export class BoardManagerService {
    private backend: BoardManagerBackend;

    constructor(
        @inject(WebSocketConnectionProvider) protected readonly connectionProvider: WebSocketConnectionProvider
    ) {
        this.backend = this.connectionProvider.createProxy<BoardManagerBackend>(BoardManagerBackendPath);
    }

    async detectBoards(): Promise<BoardInfo[]> {
        try {
            return await this.backend.detectBoards();
        } catch (error) {
            console.error('Failed to detect boards:', error);
            return [];
        }
    }

    async getSerialPorts(): Promise<string[]> {
        try {
            return await this.backend.getSerialPorts();
        } catch (error) {
            console.error('Failed to get serial ports:', error);
            return [];
        }
    }

    async uploadCode(code: string, port: string, fqbn: string): Promise<{ success: boolean; message: string }> {
        try {
            return await this.backend.uploadCode(code, port, fqbn);
        } catch (error) {
            console.error('Failed to upload code:', error);
            return { success: false, message: error instanceof Error ? error.message : 'Unknown error' };
        }
    }

    async verifyCode(code: string, fqbn: string): Promise<{ success: boolean; message: string }> {
        try {
            return await this.backend.verifyCode(code, fqbn);
        } catch (error) {
            console.error('Failed to verify code:', error);
            return { success: false, message: error instanceof Error ? error.message : 'Unknown error' };
        }
    }
}
