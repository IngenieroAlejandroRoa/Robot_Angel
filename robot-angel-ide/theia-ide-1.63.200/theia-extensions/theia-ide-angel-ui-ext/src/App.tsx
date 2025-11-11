import * as React from "react";
import { useState, useRef } from "react";
import {
  Panel,
  PanelGroup,
  PanelResizeHandle,
} from "react-resizable-panels";

import { TopToolbar } from "./components/TopToolbar";
import { SidebarPanel } from "./components/SidebarPanel";
import { CodeEditor } from "./components/CodeEditor";
import { Terminal } from "./components/Terminal";
import { SerialMonitor } from "./components/SerialMonitor";

export default function App() {
  const [isSerialMonitorVisible, setIsSerialMonitorVisible] = useState(false);
  const [currentFile, setCurrentFile] = useState<string | null>(null);
  const [fileContent, setFileContent] = useState<string>("");
  const [isModified, setIsModified] = useState(false);
  
  const codeEditorRef = useRef<any>(null);

  const toggleSerialMonitor = () => {
    setIsSerialMonitorVisible(!isSerialMonitorVisible);
  };

  const handleOpenFile = async () => {
    try {
      // @ts-ignore - Angel File Service
      const angelFileService = window.angelFileService;
      if (!angelFileService) {
        console.error('Angel file service not available');
        return;
      }

      const result = await angelFileService.openFile();
      if (result && result.filePath && result.content) {
        setCurrentFile(result.filePath);
        setFileContent(result.content);
        setIsModified(false);
      }
    } catch (error) {
      console.error('Error opening file:', error);
    }
  };

  const handleSaveFile = async () => {
    // @ts-ignore - Angel File Service
    const angelFileService = window.angelFileService;
    if (!angelFileService) {
      console.error('Angel file service not available');
      return;
    }

    if (!currentFile) {
      await handleSaveAsFile();
      return;
    }

    try {
      await angelFileService.saveFile(currentFile, fileContent);
      setIsModified(false);
    } catch (error) {
      console.error('Error saving file:', error);
    }
  };

  const handleSaveAsFile = async () => {
    try {
      // @ts-ignore - Angel File Service
      const angelFileService = window.angelFileService;
      if (!angelFileService) {
        console.error('Angel file service not available');
        return;
      }

      const filePath = await angelFileService.saveFileAs(fileContent);
      if (filePath) {
        setCurrentFile(filePath);
        setIsModified(false);
      }
    } catch (error) {
      console.error('Error saving file as:', error);
    }
  };

  const handleNewFile = () => {
    setCurrentFile(null);
    setFileContent("");
    setIsModified(false);
  };

  const handleContentChange = (content: string) => {
    setFileContent(content);
    setIsModified(true);
  };

  return (
    <div className="h-screen bg-gray-900 flex flex-col dark">
      {/* --- TOP TOOLBAR --- */}
      <TopToolbar
        onSerialMonitorToggle={toggleSerialMonitor}
        isSerialMonitorVisible={isSerialMonitorVisible}
        onOpenFile={handleOpenFile}
        onSaveFile={handleSaveFile}
        onNewFile={handleNewFile}
        isModified={isModified}
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
                <CodeEditor 
                  ref={codeEditorRef}
                  content={fileContent}
                  filePath={currentFile}
                  onContentChange={handleContentChange}
                />
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
}
