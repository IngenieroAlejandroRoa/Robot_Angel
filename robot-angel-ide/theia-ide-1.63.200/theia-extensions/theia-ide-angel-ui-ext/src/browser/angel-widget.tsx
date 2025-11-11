/*
 * Copyright (c) 2025 Robot Angel project authors.
 *
 * This file defines the main Theia widget which hosts the
 * custom Robot Angel UI.  The widget extends the built‑in
 * `ReactWidget` base class provided by Theia.  ReactWidget
 * handles the DOM lifecycle on our behalf and allows us to
 * return JSX directly from the `render` method.  See the
 * official Theia documentation for details on writing React
 * widgets【494939386884266†L146-L163】.
 */

import * as React from 'react';
import { injectable, postConstruct, inject } from '@theia/core/shared/inversify';
import { ReactWidget } from '@theia/core/lib/browser/widgets/react-widget';

// Import the root React component of the custom UI.  We keep
// the component unchanged so that the same UI can be used both
// standalone (with Vite) and inside Theia.  The default export
// from App.tsx exposes the complete layout defined by the
// designer.
import App from '../App';
// Pull in the global Tailwind CSS so that the UI is styled correctly.
import '../index.css';
// Import Theia-specific overrides
import '../theia-integration.css';
// Import terminal service and hook initializer
import { AngelTerminalService } from './terminal-service';
import { setTerminalServiceInstance } from '../hooks/useTerminalService';
import { AngelFileService } from './angel-file-service';

/**
 * AngelWidget wraps the custom React application into a Theia
 * widget.  The widget is identified by a static `ID` and
 * `LABEL`.  The framework uses these identifiers to manage
 * instances of the view and to display a label in the tab bar.
 */
@injectable()
export class AngelWidget extends ReactWidget {
    @inject(AngelTerminalService)
    protected readonly terminalService: AngelTerminalService;

    @inject(AngelFileService)
    protected readonly fileService: AngelFileService;

    /**
     * A unique identifier for this widget.  When adding
     * contributions (see AngelWidgetContribution) you must
     * reference this ID.
     */
    // Use only ASCII characters in the widget ID to avoid issues with non-standard hyphens.
    static readonly ID = 'robot-angel-ui:widget';

    /**
     * A human readable name for the widget.  This text will
     * appear in the Theia UI, for example in the view menu and
     * the tab bar.
     */
    // Avoid non-breaking space in the label.
    static readonly LABEL = 'Robot Angel';

    /**
     * Initialise the widget.  The `@postConstruct` decorator
     * ensures that this method is called after the dependency
     * injection container has set up the instance.  Here we
     * configure the widget’s id, title and other metadata and
     * trigger an initial render by calling `update()`.
     */
    @postConstruct()
    protected async init(): Promise<void> {
        console.log('AngelWidget init() called');
        
        // IMPORTANT: Expose terminal service globally FIRST
        (window as any).angelTerminalService = this.terminalService;
        console.log('Terminal service exposed globally:', this.terminalService);
        
        // Expose file service globally
        (window as any).angelFileService = this.fileService;
        console.log('File service exposed globally:', this.fileService);
        
        // Also initialize for React hooks
        setTerminalServiceInstance(this.terminalService);
        
        this.id = AngelWidget.ID;
        this.title.label = AngelWidget.LABEL;
        this.title.caption = AngelWidget.LABEL;
        this.title.closable = false;
        this.title.iconClass = 'fa fa-microchip';
        
        // Asegurar que el widget ocupe todo el espacio disponible
        this.node.style.width = '100%';
        this.node.style.height = '100%';
        this.node.style.overflow = 'auto';
        this.node.style.display = 'block';
        this.node.style.position = 'relative';
        this.node.style.background = '#1a1a2e';
        
        // Agregar clase para identificación
        this.node.classList.add('robot-angel-widget');
        
        console.log('AngelWidget calling update()');
        console.log('Widget isVisible:', this.isVisible);
        console.log('Widget isAttached:', this.isAttached);
        this.update();
        
        // Forzar render después de un momento
        setTimeout(() => {
            console.log('Forcing update after timeout');
            this.update();
        }, 100);
    }

    protected onAfterAttach(msg: any): void {
        super.onAfterAttach(msg);
        console.log('AngelWidget onAfterAttach called');
        console.log('Widget isVisible:', this.isVisible);
        console.log('Widget isAttached:', this.isAttached);
        // Forzar un re-render después de adjuntar
        setTimeout(() => {
            console.log('Forcing update after attach');
            this.update();
        }, 50);
    }
    
    protected onActivateRequest(msg: any): void {
        super.onActivateRequest(msg);
        console.log('AngelWidget onActivateRequest called');
        this.update();
    }

    /**
     * Render the React application.  `ReactWidget` will call
     * this method whenever the widget needs to update.  The
     * returned JSX will be diffed and rendered into the widget’s
     * DOM node.  Note that we do not manipulate the DOM directly
     * here – React manages the contents of the widget’s node.
     */
    protected render(): React.ReactNode {
        console.log("AngelWidget render() called - rendering full App");
        try {
            return <App />;
        } catch (error) {
            console.error('Error rendering App:', error);
            return (
                <div style={{ 
                    width: "100%", 
                    height: "100%", 
                    display: "flex", 
                    alignItems: "center", 
                    justifyContent: "center",
                    background: "#1a1a2e",
                    color: "white",
                    fontSize: "24px"
                }}>
                    <div>
                        <h1>Error al cargar Robot Angel UI</h1>
                        <p>{String(error)}</p>
                    </div>
                </div>
            );
        }
    }
}