/*
 * Copyright (c) 2025 Robot Angel project authors.
 *
 * This file defines a view contribution for the Robot Angel
 * widget.  Contributions wire our widget into Theia by
 * registering commands and specifying default layout options.
 * See the Theia documentation on widget contributions for
 * details【494939386884266†L146-L174】.
 */

import { injectable, inject } from '@theia/core/shared/inversify';
import { FrontendApplication, FrontendApplicationContribution, WidgetManager } from '@theia/core/lib/browser';
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
    // Use only ASCII characters in the command ID and label.
    id: 'robot-angel-ui:open',
    label: 'Open Robot Angel UI',
};

/**
 * AngelWidgetContribution wires the custom widget into the
 * workbench.  By extending `AbstractViewContribution` we get
 * default behaviour to add the view to the View menu and a
 * convenient `openView()` helper method.
 */
@injectable()
export class AngelWidgetContribution extends AbstractViewContribution<AngelWidget> {
    
    @inject(WidgetManager)
    protected readonly widgetManager!: WidgetManager;
    
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
            execute: () => this.openView({ activate: true, reveal: true }),
        });
    }

    /**
     * Called when the frontend application starts.
     * We use this to automatically open the Robot Angel UI.
     */
    async onStart(app: FrontendApplication): Promise<void> {
        console.log('AngelWidgetContribution onStart() called');
        
        // Wait for the shell to be fully attached and ready
        await app.shell.pendingUpdates;
        
        // Give the application MORE time to settle (increased from 1000ms)
        setTimeout(async () => {
            console.log('Opening Angel widget after timeout...');
            
            try {
                // Close any existing main area widgets
                const mainWidgets = app.shell.getWidgets('main');
                console.log('Main widgets found:', mainWidgets.length);
                
                for (const widget of mainWidgets) {
                    console.log('Closing widget:', widget.id);
                    widget.close();
                }
                
                // Create widget directly using widget manager
                console.log('Creating widget via widgetManager (ASYNC)...');
                const widget = await this.widgetManager.getOrCreateWidget(AngelWidget.ID) as AngelWidget;
                console.log('Widget created:', widget);
                console.log('Widget ID:', widget?.id);
                
                if (!widget) {
                    console.error('Widget creation returned null/undefined!');
                    return;
                }
                
                // Add widget to main area
                console.log('Adding widget to main area...');
                app.shell.addWidget(widget, { area: 'main' });
                
                // Activate widget
                console.log('Activating widget...');
                await app.shell.activateWidget(widget.id);
                
                console.log('Widget isVisible:', widget.isVisible);
                console.log('Widget isAttached:', widget.isAttached);
                console.log('Widget parent:', widget.parent);
                console.log('✅ Robot Angel UI opened successfully!');
            } catch (error) {
                console.error('Error opening Angel widget:', error);
            }
        }, 2000); // Increased from 1000ms to 2000ms
    }
}