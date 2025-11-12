/*
 * Copyright (c) 2025 Robot Angel project authors.
 *
 * This module binds our widget and contribution into Theia's
 * dependency injection container.  It will be discovered via
 * the `theiaExtensions` entry in package.json.  When the
 * frontend starts, Theia will load this container module and
 * execute the bindings below.
 */

import { ContainerModule } from '@theia/core/shared/inversify';
import { bindViewContribution, FrontendApplicationContribution, WebSocketConnectionProvider } from '@theia/core/lib/browser';
import { WidgetFactory } from '@theia/core/lib/browser/widget-manager';
import { AngelWidget } from './angel-widget';
import { AngelWidgetContribution } from './angel-contribution';
import { AngelTerminalService } from './terminal-service';
import { AngelFileService } from './angel-file-service';
import { TerminalBackend, TerminalBackendPath } from '../common/terminal-protocol';

export default new ContainerModule(bind => {
    // Register our contribution so it becomes available in the
    // View menu and command palette.
    bindViewContribution(bind, AngelWidgetContribution);
    
    // IMPORTANT: Also bind as FrontendApplicationContribution to ensure
    // initializeLayout() is called
    bind(FrontendApplicationContribution).toService(AngelWidgetContribution);

    // Bind the terminal service
    bind(AngelTerminalService).toSelf().inSingletonScope();

    // Bind the file service
    bind(AngelFileService).toSelf().inSingletonScope();

    // Bind the terminal backend RPC
    bind(TerminalBackend).toDynamicValue(ctx => {
        const connection = ctx.container.get(WebSocketConnectionProvider);
        return connection.createProxy<TerminalBackend>(TerminalBackendPath);
    }).inSingletonScope();

    // Bind the widget so that the DI container can create it.
    bind(AngelWidget).toSelf().inSingletonScope();

    // Register a widget factory for our widget.  The widget
    // manager uses factories to lazily create widgets on demand.
    bind(WidgetFactory).toDynamicValue(ctx => ({
        id: AngelWidget.ID,
        createWidget: () => Promise.resolve(ctx.container.get<AngelWidget>(AngelWidget)),
    })).inSingletonScope();
});