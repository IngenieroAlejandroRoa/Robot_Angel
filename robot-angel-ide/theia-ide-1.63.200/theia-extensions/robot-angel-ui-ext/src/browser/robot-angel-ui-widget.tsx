/********************************************************************************
 * Copyright (C) 2025 Robot Angel Project
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License, which is available in the project root.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************************************/

import * as React from 'react';
import { injectable, postConstruct } from '@theia/core/shared/inversify';
import { ReactWidget } from '@theia/core/lib/browser/widgets/react-widget';
import { RobotAngelUIApp } from './robot-angel-ui-app';

@injectable()
export class RobotAngelUIWidget extends ReactWidget {

    static readonly ID = 'robot-angel-ui:widget';
    static readonly LABEL = 'Robot Angel UI';

    @postConstruct()
    protected init(): void {
        this.id = RobotAngelUIWidget.ID;
        this.title.label = RobotAngelUIWidget.LABEL;
        this.title.caption = RobotAngelUIWidget.LABEL;
        this.title.closable = true;
        this.title.iconClass = 'fa fa-robot';
        this.update();
    }

    protected render(): React.ReactNode {
        return <RobotAngelUIApp />;
    }
}
