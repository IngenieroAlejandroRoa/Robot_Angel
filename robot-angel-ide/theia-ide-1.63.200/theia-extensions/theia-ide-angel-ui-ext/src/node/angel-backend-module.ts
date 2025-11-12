import { ContainerModule } from '@theia/core/shared/inversify';
import { ConnectionHandler, RpcConnectionHandler } from '@theia/core';
import { TerminalBackendImpl, TerminalBackendPath } from './terminal-backend';
import { TerminalBackend } from '../common/terminal-protocol';

export default new ContainerModule(bind => {
    bind(TerminalBackendImpl).toSelf().inSingletonScope();
    bind(ConnectionHandler).toDynamicValue(ctx =>
        new RpcConnectionHandler(TerminalBackendPath, () => {
            return ctx.container.get<TerminalBackendImpl>(TerminalBackendImpl);
        })
    ).inSingletonScope();
});
