import { injectable, inject } from '@theia/core/shared/inversify';
import { FileService } from '@theia/filesystem/lib/browser/file-service';
import { FileDialogService, OpenFileDialogProps } from '@theia/filesystem/lib/browser';
import { WorkspaceService } from '@theia/workspace/lib/browser';
import URI from '@theia/core/lib/common/uri';

@injectable()
export class AngelFileService {

    @inject(FileService)
    protected readonly fileService: FileService;

    @inject(FileDialogService)
    protected readonly fileDialogService: FileDialogService;

    @inject(WorkspaceService)
    protected readonly workspaceService: WorkspaceService;

    async openFile(): Promise<{ filePath: string; content: string } | null> {
        const props: OpenFileDialogProps = {
            title: 'Open File',
            canSelectFiles: true,
            canSelectFolders: false,
            canSelectMany: false,
        };
        
        const uri = await this.fileDialogService.showOpenDialog(props);

        if (!uri) {
            return null;
        }

        try {
            const fileContent = await this.fileService.read(uri);
            return {
                filePath: uri.path.toString(),
                content: fileContent.value
            };
        } catch (error) {
            console.error('Error reading file:', error);
            return null;
        }
    }

    async saveFile(filePath: string, content: string): Promise<boolean> {
        try {
            const uri = new URI(filePath);
            await this.fileService.write(uri, content);
            return true;
        } catch (error) {
            console.error('Error saving file:', error);
            return false;
        }
    }

    async saveFileAs(content: string): Promise<string | null> {
        const uri = await this.fileDialogService.showSaveDialog({
            title: 'Save File As',
        });

        if (!uri) {
            return null;
        }

        try {
            await this.fileService.write(uri, content);
            return uri.path.toString();
        } catch (error) {
            console.error('Error saving file:', error);
            return null;
        }
    }

    async createNewFile(content: string = ''): Promise<string | null> {
        const uri = await this.fileDialogService.showSaveDialog({
            title: 'New File',
        });

        if (!uri) {
            return null;
        }

        try {
            await this.fileService.create(uri, content);
            return uri.path.toString();
        } catch (error) {
            console.error('Error creating file:', error);
            return null;
        }
    }
}
