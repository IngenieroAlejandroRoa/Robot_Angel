import { ContainerModule } from '@theia/core/shared/inversify';
import { ConnectionHandler, RpcConnectionHandler } from '@theia/core';
import { TerminalBackendImpl, TerminalBackendPath } from './terminal-backend';
import { TerminalBackend } from '../common/terminal-protocol';
import { BoardManagerBackend } from './board-manager-backend';
import { SerialBackendImpl, SerialBackendPath } from './serial-backend';

export const BoardManagerBackendPath = '/services/board-manager';

export default new ContainerModule(bind => {
    bind(TerminalBackendImpl).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new RpcConnectionHandler(TerminalBackendPath, () => {
            return ctx.container.get<TerminalBackendImpl>(TerminalBackendImpl);
        })
    ).inSingletonScope();
    
    bind(BoardManagerBackend).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new RpcConnectionHandler(BoardManagerBackendPath, () => {
            return ctx.container.get<BoardManagerBackend>(BoardManagerBackend);
        })
    ).inSingletonScope();
    
    bind(SerialBackendImpl).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new RpcConnectionHandler(SerialBackendPath, () => {
            return ctx.container.get<SerialBackendImpl>(SerialBackendImpl);
        })
    ).inSingletonScope();
});
