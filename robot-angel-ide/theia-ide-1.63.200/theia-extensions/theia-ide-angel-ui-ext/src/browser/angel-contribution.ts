/*
 * Copyright (c) 2025 Robot Angel project authors.
 *
 * This file defines a view contribution for the Robot Angel
 * widget.  Contributions wire our widget into Theia by
 * registering commands and specifying default layout options.
 * See the Theia documentation on widget contributions for
 * details【494939386884266†L146-L174】.
 */

import { injectable } from '@theia/core/shared/inversify';
import {
    AbstractViewContribution,
    WidgetFactory,
} from '@theia/core/lib/browser';
import { Command, CommandRegistry } from '@theia/core/lib/common';
import { AngelWidget } from './angel-widget';

/**
 * A command used to open or toggle the Robot Angel UI.  The
 * label will appear in command palettes and menus.
 */
export const AngelCommand: Command = {
    id: 'robot‑angel‑ui:open',
    label: 'Open Robot Angel UI',
};

/**
 * AngelWidgetContribution wires the custom widget into the
 * workbench.  By extending `AbstractViewContribution` we get
 * default behaviour to add the view to the View menu and a
 * convenient `openView()` helper method.
 */
@injectable()
export class AngelWidgetContribution extends AbstractViewContribution<AngelWidget> {
    constructor() {
        super({
            widgetId: AngelWidget.ID,
            widgetName: AngelWidget.LABEL,
            defaultWidgetOptions: {
                area: 'main',
            },
            toggleCommandId: AngelCommand.id,
        });
    }

    /**
     * Register the command that opens our view.  Using
     * `super.openView` ensures the widget is created via the
     * widget manager and placed according to the configured
     * `defaultWidgetOptions`.
     */
    registerCommands(commands: CommandRegistry): void {
        commands.registerCommand(AngelCommand, {
            execute: () => this.openView({ activate: true }),
        });
    }
}