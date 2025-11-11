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

interface FileTab {
  id: string;
  filePath: string;
  content: string;
  isModified: boolean;
}

export default function App() {
  const [isSerialMonitorVisible, setIsSerialMonitorVisible] = useState(false);
  const [openFiles, setOpenFiles] = useState<FileTab[]>([]);
  const [activeFileId, setActiveFileId] = useState<string | null>(null);
  
  const codeEditorRef = useRef<any>(null);

  const activeFile = openFiles.find(f => f.id === activeFileId);

  const toggleSerialMonitor = () => {
    setIsSerialMonitorVisible(!isSerialMonitorVisible);
  };

  const generateFileId = (filePath: string): string => {
    return `file-${Date.now()}-${filePath.replace(/[^a-zA-Z0-9]/g, '-')}`;
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
        // Check if file is already open
        const existingFile = openFiles.find(f => f.filePath === result.filePath);
        if (existingFile) {
          setActiveFileId(existingFile.id);
          return;
        }

        // Add new file tab
        const newFile: FileTab = {
          id: generateFileId(result.filePath),
          filePath: result.filePath,
          content: result.content,
          isModified: false,
        };

        setOpenFiles([...openFiles, newFile]);
        setActiveFileId(newFile.id);
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

    if (!activeFile) {
      await handleSaveAsFile();
      return;
    }

    try {
      await angelFileService.saveFile(activeFile.filePath, activeFile.content);
      
      // Update modified state
      setOpenFiles(openFiles.map(f => 
        f.id === activeFileId ? { ...f, isModified: false } : f
      ));
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

      const content = activeFile?.content || '';
      const filePath = await angelFileService.saveFileAs(content);
      
      if (filePath) {
        if (activeFile) {
          // Update existing file
          setOpenFiles(openFiles.map(f =>
            f.id === activeFileId
              ? { ...f, filePath, isModified: false }
              : f
          ));
        } else {
          // Create new file
          const newFile: FileTab = {
            id: generateFileId(filePath),
            filePath,
            content,
            isModified: false,
          };
          setOpenFiles([...openFiles, newFile]);
          setActiveFileId(newFile.id);
        }
      }
    } catch (error) {
      console.error('Error saving file as:', error);
    }
  };

  const handleNewFile = () => {
    const newFile: FileTab = {
      id: generateFileId('untitled'),
      filePath: '',
      content: '',
      isModified: false,
    };
    setOpenFiles([...openFiles, newFile]);
    setActiveFileId(newFile.id);
  };

  const handleContentChange = (content: string) => {
    if (!activeFileId) return;

    setOpenFiles(openFiles.map(f =>
      f.id === activeFileId
        ? { ...f, content, isModified: true }
        : f
    ));
  };

  const handleCloseFile = (fileId: string) => {
    const updatedFiles = openFiles.filter(f => f.id !== fileId);
    setOpenFiles(updatedFiles);

    // If closing active file, switch to another
    if (fileId === activeFileId) {
      if (updatedFiles.length > 0) {
        setActiveFileId(updatedFiles[updatedFiles.length - 1].id);
      } else {
        setActiveFileId(null);
      }
    }
  };

  const handleTabClick = (fileId: string) => {
    setActiveFileId(fileId);
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
        isModified={activeFile?.isModified || false}
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
                  content={activeFile?.content || ''}
                  filePath={activeFile?.filePath || null}
                  onContentChange={handleContentChange}
                  openFiles={openFiles}
                  activeFileId={activeFileId}
                  onTabClick={handleTabClick}
                  onCloseTab={handleCloseFile}
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
