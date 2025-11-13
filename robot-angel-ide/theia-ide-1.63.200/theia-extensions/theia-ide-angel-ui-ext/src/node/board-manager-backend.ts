import { injectable } from '@theia/core/shared/inversify';
import { spawn, ChildProcess } from 'child_process';
import * as path from 'path';
import * as fs from 'fs';
import * as os from 'os';

export interface BoardInfo {
    port: string;
    boardType: string;
    fqbn: string;
}

export interface UploadProgress {
    stage: 'compiling' | 'uploading' | 'done' | 'error';
    message: string;
}

@injectable()
export class BoardManagerBackend {
    private pythonPath: string;
    private scriptPath: string;

    constructor() {
        // Find Python from conda environment
        const condaPrefix = process.env.CONDA_PREFIX || path.join(os.homedir(), 'miniconda3', 'envs', 'robot-angel');
        this.pythonPath = path.join(condaPrefix, 'bin', 'python');
        
        // Path to Robot Angel Python utilities
        this.scriptPath = path.join(os.homedir(), 'Desktop', 'RobotAngel', 'robot_angel');
    }

    async detectBoards(): Promise<BoardInfo[]> {
        return new Promise((resolve, reject) => {
            const script = `
import sys
sys.path.insert(0, '${this.scriptPath}')
from utils.board_manager import BoardManager
import json

manager = BoardManager()
boards = manager.detect_boards()

result = []
for board in boards:
    result.append({
        'port': board.port,
        'boardType': board.board_type,
        'fqbn': board.fqbn
    })

print(json.dumps(result))
`;

            const proc = spawn(this.pythonPath, ['-c', script]);
            let stdout = '';
            let stderr = '';

            proc.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            proc.stderr.on('data', (data) => {
                stderr += data.toString();
            });

            proc.on('close', (code) => {
                if (code === 0) {
                    try {
                        const boards = JSON.parse(stdout.trim());
                        resolve(boards);
                    } catch (e) {
                        console.error('Failed to parse board detection output:', stdout);
                        resolve([]);
                    }
                } else {
                    console.error('Board detection failed:', stderr);
                    resolve([]);
                }
            });

            proc.on('error', (err) => {
                console.error('Failed to spawn board detection:', err);
                resolve([]);
            });
        });
    }

    async getSerialPorts(): Promise<string[]> {
        return new Promise((resolve) => {
            const proc = spawn('sh', ['-c', 'ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true']);
            let stdout = '';

            proc.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            proc.on('close', () => {
                const ports = stdout.trim().split('\n').filter(p => p);
                resolve(ports);
            });

            proc.on('error', () => {
                resolve([]);
            });
        });
    }

    async uploadCode(
        code: string,
        port: string,
        fqbn: string,
        onProgress?: (progress: UploadProgress) => void
    ): Promise<{ success: boolean; message: string }> {
        return new Promise((resolve) => {
            // Create temporary file for the code
            const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-angel-upload-'));
            const codeFile = path.join(tmpDir, 'code.ino');
            fs.writeFileSync(codeFile, code);

            const script = `
import sys
sys.path.insert(0, '${this.scriptPath}')
from utils.arduino_uploader import ArduinoUploader
from utils.board_manager import BoardManager
import json

manager = BoardManager()
uploader = ArduinoUploader(manager.arduino_cli or 'arduino-cli')

def progress_callback(msg):
    print('PROGRESS:', msg, flush=True)

with open('${codeFile}', 'r') as f:
    code = f.read()

success, message = uploader.compile_and_upload(
    code,
    '${port}',
    '${fqbn}',
    progress_callback
)

result = {'success': success, 'message': message}
print('RESULT:', json.dumps(result))
`;

            const proc = spawn(this.pythonPath, ['-c', script]);
            let resultData = '';

            proc.stdout.on('data', (data) => {
                const output = data.toString();
                const lines = output.split('\n');
                
                for (const line of lines) {
                    if (line.startsWith('PROGRESS:')) {
                        const msg = line.substring(9).trim();
                        if (onProgress) {
                            if (msg.includes('Compiling')) {
                                onProgress({ stage: 'compiling', message: msg });
                            } else if (msg.includes('Uploading')) {
                                onProgress({ stage: 'uploading', message: msg });
                            } else if (msg.includes('successful')) {
                                onProgress({ stage: 'done', message: msg });
                            } else if (msg.includes('failed')) {
                                onProgress({ stage: 'error', message: msg });
                            }
                        }
                    } else if (line.startsWith('RESULT:')) {
                        resultData = line.substring(7).trim();
                    }
                }
            });

            proc.stderr.on('data', (data) => {
                console.error('Upload error:', data.toString());
            });

            proc.on('close', (code) => {
                // Cleanup
                try {
                    fs.unlinkSync(codeFile);
                    fs.rmdirSync(tmpDir);
                } catch (e) {
                    // Ignore cleanup errors
                }

                if (resultData) {
                    try {
                        const result = JSON.parse(resultData);
                        resolve(result);
                    } catch (e) {
                        resolve({ success: false, message: 'Failed to parse result' });
                    }
                } else {
                    resolve({ success: false, message: 'Upload process failed' });
                }
            });

            proc.on('error', (err) => {
                console.error('Failed to spawn upload process:', err);
                resolve({ success: false, message: err.message });
            });
        });
    }

    async verifyCode(code: string, fqbn: string): Promise<{ success: boolean; message: string }> {
        return new Promise((resolve) => {
            const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), 'robot-angel-verify-'));
            const codeFile = path.join(tmpDir, 'code.ino');
            fs.writeFileSync(codeFile, code);

            const script = `
import sys
sys.path.insert(0, '${this.scriptPath}')
from utils.arduino_uploader import ArduinoUploader
from utils.board_manager import BoardManager
import json

manager = BoardManager()
uploader = ArduinoUploader(manager.arduino_cli or 'arduino-cli')

with open('${codeFile}', 'r') as f:
    code = f.read()

success, message = uploader.verify_only(code, '${fqbn}')
result = {'success': success, 'message': message}
print(json.dumps(result))
`;

            const proc = spawn(this.pythonPath, ['-c', script]);
            let stdout = '';

            proc.stdout.on('data', (data) => {
                stdout += data.toString();
            });

            proc.on('close', () => {
                try {
                    fs.unlinkSync(codeFile);
                    fs.rmdirSync(tmpDir);
                } catch (e) {
                    // Ignore
                }

                try {
                    const result = JSON.parse(stdout.trim());
                    resolve(result);
                } catch (e) {
                    resolve({ success: false, message: 'Verification failed' });
                }
            });

            proc.on('error', (err) => {
                resolve({ success: false, message: err.message });
            });
        });
    }
}
