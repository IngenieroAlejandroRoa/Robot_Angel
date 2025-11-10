/********************************************************************************
 * Copyright (C) 2025 Robot Angel Project
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License, which is available in the project root.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************************************/

import * as React from 'react';
import { useState } from 'react';
import {
    Panel,
    PanelGroup,
    PanelResizeHandle,
} from 'react-resizable-panels';

import { TopToolbar } from './components/TopToolbar';
import { SidebarPanel } from './components/SidebarPanel';
import { CodeEditor } from './components/CodeEditor';
import { Terminal } from './components/Terminal';
import { SerialMonitor } from './components/SerialMonitor';

import './index.css';

export const RobotAngelUIApp = () => {
    const [isSerialMonitorVisible, setIsSerialMonitorVisible] = useState(false);

    const toggleSerialMonitor = () => {
        setIsSerialMonitorVisible(!isSerialMonitorVisible);
    };

    return (
        <div className="h-full bg-gray-900 flex flex-col dark">
            {/* --- TOP TOOLBAR --- */}
            <TopToolbar
                onSerialMonitorToggle={toggleSerialMonitor}
                isSerialMonitorVisible={isSerialMonitorVisible}
            />

            {/* --- MAIN LAYOUT: Sidebar | Editor/Monitor | Terminal --- */}
            <PanelGroup
                direction="horizontal"
                className="flex-1 min-h-0 overflow-hidden"
            >
                {/* --- SIDEBAR --- */}
                <Panel
                    defaultSize={20}
                    minSize={10}
                    maxSize={30}
                    className="min-h-0 overflow-hidden"
                >
                    <SidebarPanel />
                </Panel>

                <PanelResizeHandle className="w-[4px] bg-gray-700 hover:bg-purple-500 transition-colors" />

                {/* --- CENTER AREA (Editor + Serial Monitor) --- */}
                <Panel className="flex-1 min-h-0 overflow-hidden">
                    <PanelGroup
                        direction="vertical"
                        className="min-h-0 flex-1 overflow-hidden"
                    >
                        {/* CODE EDITOR */}
                        <Panel
                            defaultSize={70}
                            minSize={30}
                            className="min-h-0 overflow-hidden"
                        >
                            <div className="flex flex-col h-full min-h-0">
                                <CodeEditor />
                            </div>
                        </Panel>

                        {/* SERIAL MONITOR (optional toggle) */}
                        {isSerialMonitorVisible && (
                            <>
                                <PanelResizeHandle className="h-[4px] bg-gray-700 hover:bg-purple-500 transition-colors" />
                                <Panel
                                    defaultSize={30}
                                    minSize={20}
                                    maxSize={50}
                                    className="min-h-0 overflow-hidden"
                                >
                                    <SerialMonitor />
                                </Panel>
                            </>
                        )}
                    </PanelGroup>
                </Panel>

                <PanelResizeHandle className="w-[4px] bg-gray-700 hover:bg-purple-500 transition-colors" />

                {/* --- TERMINAL --- */}
                <Panel
                    defaultSize={25}
                    minSize={20}
                    maxSize={40}
                    className="min-h-0 overflow-hidden"
                >
                    <Terminal />
                </Panel>
            </PanelGroup>
        </div>
    );
};
