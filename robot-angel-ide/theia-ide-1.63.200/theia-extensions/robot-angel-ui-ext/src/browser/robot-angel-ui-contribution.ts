/********************************************************************************
 * Copyright (C) 2025 Robot Angel Project
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License, which is available in the project root.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************************************/

import { injectable } from '@theia/core/shared/inversify';
import { Command, CommandContribution, CommandRegistry, MenuContribution, MenuModelRegistry } from '@theia/core/lib/common';
import { CommonMenus, AbstractViewContribution } from '@theia/core/lib/browser';
import { RobotAngelUIWidget } from './robot-angel-ui-widget';

export const RobotAngelUICommand: Command = {
    id: 'robot-angel-ui:toggle',
    label: 'Toggle Robot Angel UI'
};

@injectable()
export class RobotAngelUIContribution extends AbstractViewContribution<RobotAngelUIWidget> implements CommandContribution, MenuContribution {

    constructor() {
        super({
            widgetId: RobotAngelUIWidget.ID,
            widgetName: RobotAngelUIWidget.LABEL,
            defaultWidgetOptions: {
                area: 'main'
            },
            toggleCommandId: RobotAngelUICommand.id
        });
    }

    registerCommands(commands: CommandRegistry): void {
        commands.registerCommand(RobotAngelUICommand, {
            execute: () => this.openView({ activate: true, reveal: true })
        });
    }

    registerMenus(menus: MenuModelRegistry): void {
        menus.registerMenuAction(CommonMenus.VIEW_VIEWS, {
            commandId: RobotAngelUICommand.id,
            label: RobotAngelUICommand.label
        });
    }
}
