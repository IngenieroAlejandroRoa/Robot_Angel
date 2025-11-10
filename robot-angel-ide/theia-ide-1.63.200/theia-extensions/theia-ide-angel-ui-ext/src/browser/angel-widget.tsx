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
import { injectable, postConstruct } from '@theia/core/shared/inversify';
import { ReactWidget } from '@theia/core/lib/browser/widgets/react-widget';

// Import the root React component of the custom UI.  We keep
// the component unchanged so that the same UI can be used both
// standalone (with Vite) and inside Theia.  The default export
// from App.tsx exposes the complete layout defined by the
// designer.
import App from '../App';
// Pull in the global Tailwind CSS so that the UI is styled correctly.

/**
 * AngelWidget wraps the custom React application into a Theia
 * widget.  The widget is identified by a static `ID` and
 * `LABEL`.  The framework uses these identifiers to manage
 * instances of the view and to display a label in the tab bar.
 */
@injectable()
export class AngelWidget extends ReactWidget {
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
        this.id = AngelWidget.ID;
        this.title.label = AngelWidget.LABEL;
        this.title.caption = AngelWidget.LABEL;
        this.title.closable = true;
        // Optionally set an icon.  The class must follow the
        // naming scheme used by Theia (e.g. 'fa fa‑robot').  You
        // can define your own CSS icon classes if needed.
        this.title.iconClass = 'fa fa‑microchip';
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
        return <App />;
    }
}