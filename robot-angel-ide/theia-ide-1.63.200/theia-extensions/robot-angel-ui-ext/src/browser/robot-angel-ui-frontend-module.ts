/********************************************************************************
 * Copyright (C) 2025 Robot Angel Project
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License, which is available in the project root.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************************************/

import { ContainerModule } from '@theia/core/shared/inversify';
import { RobotAngelUIWidget } from './robot-angel-ui-widget';
import { RobotAngelUIContribution } from './robot-angel-ui-contribution';
import { bindViewContribution, FrontendApplicationContribution, WidgetFactory } from '@theia/core/lib/browser';

import './index.css';

export default new ContainerModule(bind => {
    bindViewContribution(bind, RobotAngelUIContribution);
    bind(FrontendApplicationContribution).toService(RobotAngelUIContribution);
    bind(RobotAngelUIWidget).toSelf();
    bind(WidgetFactory).toDynamicValue(ctx => ({
        id: RobotAngelUIWidget.ID,
        createWidget: () => ctx.container.get<RobotAngelUIWidget>(RobotAngelUIWidget)
    })).inSingletonScope();
});
