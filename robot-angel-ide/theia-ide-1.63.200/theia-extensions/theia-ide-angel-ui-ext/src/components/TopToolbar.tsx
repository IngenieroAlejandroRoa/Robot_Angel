import * as React from "react";
import { useState, useEffect } from "react";
import { Button } from "./ui/button";
import { Separator } from "./ui/separator";
import { Play, Square, Bug, Settings, FileText, FolderOpen, Save, Zap, Cpu, Upload, ChevronDown } from "lucide-react";
import logoImage from './assets/0619f22ea1d44864927116629fbe5ab4c93093fe.png';

interface BoardInfo {
  port: string;
  boardType: string;
  fqbn: string;
}

interface TopToolbarProps {
  onSerialMonitorToggle: () => void;
  isSerialMonitorVisible: boolean;
  onOpenFile: () => void;
  onSaveFile: () => void;
  onNewFile: () => void;
  isModified: boolean;
  onRun: () => void;
  onStop: () => void;
  onDebug: () => void;
  isRunning: boolean;
  terminalRef?: React.RefObject<any>;
  onUpload?: (port: string, fqbn: string) => void;
}

export function TopToolbar({ 
  onSerialMonitorToggle, 
  isSerialMonitorVisible,
  onOpenFile,
  onSaveFile,
  onNewFile,
  isModified,
  onRun,
  onStop,
  onDebug,
  isRunning,
  terminalRef,
  onUpload
}: TopToolbarProps) {
  const [isMicroRosRunning, setIsMicroRosRunning] = useState(false);
  const [boards, setBoards] = useState<BoardInfo[]>([]);
  const [selectedBoard, setSelectedBoard] = useState<BoardInfo | null>(null);
  const [showBoardDropdown, setShowBoardDropdown] = useState(false);
  const [isUploading, setIsUploading] = useState(false);

  useEffect(() => {
    detectBoards();
    // Refresh boards every 5 seconds
    const interval = setInterval(detectBoards, 5000);
    return () => clearInterval(interval);
  }, []);

  const detectBoards = async () => {
    try {
      // @ts-ignore
      const boardManager = window.angelBoardManager;
      if (!boardManager) {
        console.error('Board manager not available');
        return;
      }

      const detectedBoards = await boardManager.detectBoards();
      setBoards(detectedBoards);
      
      // Auto-select first board if none selected
      if (detectedBoards.length > 0 && !selectedBoard) {
        setSelectedBoard(detectedBoards[0]);
      }
    } catch (error) {
      console.error('Error detecting boards:', error);
    }
  };

  const handleUpload = async () => {
    if (!selectedBoard) {
      alert('Please select a board first');
      return;
    }

    if (onUpload) {
      setIsUploading(true);
      try {
        await onUpload(selectedBoard.port, selectedBoard.fqbn);
      } finally {
        setIsUploading(false);
      }
    }
  };

  const handleMicroRosToggle = async () => {
    try {
      // @ts-ignore
      const terminalBackend = window.angelTerminalBackend;
      
      if (!terminalBackend) {
        console.error('Terminal backend not available');
        return;
      }

      if (isMicroRosRunning) {
        // Stop agent - send Ctrl+C to the specific micro-ROS terminal
        if (terminalRef && terminalRef.current && terminalRef.current.stopMicroRos) {
          await terminalRef.current.stopMicroRos();
          setIsMicroRosRunning(false);
          console.log('Micro-ROS agent stopped');
        }
      } else {
        // Start agent in new terminal
        const config = {
          transport: 'udp4',
          port: 8888,
          verbose: 6
        };
        
        // Find ROS setup and build command
        const setupPath = await terminalBackend.findRosSetup();
        if (!setupPath) {
          console.error('No ROS 2 setup found');
          return;
        }
        
        // Build the command - micro_ros_agent syntax: <transport> [options]
        let command = `source ${setupPath} && ros2 run micro_ros_agent micro_ros_agent ${config.transport}`;
        if (config.port) command += ` --port ${config.port}`;
        if (config.verbose) command += ` --verbose ${config.verbose}`;
        
        // Execute in new terminal
        if (terminalRef && terminalRef.current && terminalRef.current.executeInNewTerminal) {
          await terminalRef.current.executeInNewTerminal(command);
          setIsMicroRosRunning(true);
          console.log('Micro-ROS agent started in new terminal');
        } else {
          console.error('Terminal reference not available');
        }
      }
    } catch (error) {
      console.error('Error toggling micro-ROS agent:', error);
    }
  };

  return (
    <div className="h-12 bg-gray-900 border-b border-gray-700 flex items-center px-4">
      
      <div className="flex items-center gap-2">
        <Button 
          size="sm" 
          className={`${isRunning ? 'bg-green-600 hover:bg-green-700' : 'bg-purple-600 hover:bg-purple-700'} text-white`}
          onClick={onRun}
          disabled={isRunning}
        >
          <Play className="h-4 w-4 mr-1" />
          {isRunning ? 'Running...' : 'Run'}
        </Button>
        
        <Button 
          size="sm" 
          variant="outline"
          className="border-gray-600 bg-gray-800 hover:bg-gray-700 text-gray-200"
          onClick={onDebug}
          disabled={isRunning}
        >
          <Bug className="h-4 w-4 mr-1" />
          Debug
        </Button>
        
        <Button 
          size="sm" 
          variant="outline"
          className={`border-gray-600 ${isRunning ? 'bg-red-600 hover:bg-red-700 text-white' : 'bg-gray-800 hover:bg-gray-700 text-gray-200'}`}
          onClick={onStop}
          disabled={!isRunning}
        >
          <Square className="h-4 w-4 mr-1" />
          Stop
        </Button>
        
        <Separator orientation="vertical" className="h-6 bg-gray-600" />
        
        <Button 
          size="sm" 
          variant="ghost"
          onClick={onNewFile}
          className="text-gray-300 hover:bg-gray-700"
        >
          <FileText className="h-4 w-4 mr-1" />
          New
        </Button>
        
        <Button 
          size="sm" 
          variant="ghost"
          onClick={onOpenFile}
          className="text-gray-300 hover:bg-gray-700"
        >
          <FolderOpen className="h-4 w-4 mr-1" />
          Open
        </Button>
        
        <Button 
          size="sm" 
          variant="ghost"
          onClick={onSaveFile}
          className={`text-gray-300 hover:bg-gray-700 ${isModified ? 'text-yellow-400' : ''}`}
        >
          <Save className="h-4 w-4 mr-1" />
          Save{isModified ? '*' : ''}
        </Button>
        
        <Button 
          size="sm" 
          variant="ghost"
          onClick={onSerialMonitorToggle}
          className={`${
            isSerialMonitorVisible 
              ? "bg-purple-600 text-white hover:bg-purple-700" 
              : "text-gray-300 hover:bg-gray-700"
          }`}
        >
          <Zap className="h-4 w-4 mr-1" />
          Serial Monitor
        </Button>
        
        <Button 
          size="sm" 
          className={isMicroRosRunning 
            ? "bg-green-600 hover:bg-green-700 text-white" 
            : "bg-purple-600 hover:bg-purple-700 text-white"
          }
          onClick={handleMicroRosToggle}
          title={isMicroRosRunning ? "Stop Micro-ROS Agent" : "Start Micro-ROS Agent"}
        >
          <Cpu className="h-4 w-4 mr-1" />
          {isMicroRosRunning ? "Micro-ROS ●" : "Micro-ROS"}
        </Button>
        
        <Button 
          size="sm" 
          variant="ghost"
          className="text-gray-300 hover:bg-gray-700"
        >
          <Settings className="h-4 w-4 mr-1" />
          Settings
        </Button>
      </div>
      
      
      <div className="flex-1 flex justify-center">
        <span className="text-purple-400 font-medium text-lg">Robot Angel</span>
      </div>
      
      
      <div className="flex items-center gap-2">
        <img src={logoImage} alt="Logo" className="h-8 w-8" />
        
        {/* Board Selector Dropdown */}
        <div className="relative">
          <Button
            size="sm"
            variant="outline"
            className="border-purple-600 bg-gray-800 hover:bg-gray-700 text-gray-200"
            onClick={() => setShowBoardDropdown(!showBoardDropdown)}
          >
            <Cpu className="h-4 w-4 mr-1" />
            {selectedBoard ? selectedBoard.boardType : 'No Board'}
            <ChevronDown className="h-3 w-3 ml-1" />
          </Button>
          
          {showBoardDropdown && (
            <div className="absolute right-0 mt-1 w-64 bg-gray-800 border border-gray-600 rounded shadow-lg z-50">
              <div className="p-2">
                <div className="text-xs text-gray-400 mb-2">Available Boards:</div>
                {boards.length === 0 ? (
                  <div className="text-sm text-gray-500 p-2">No boards detected</div>
                ) : (
                  boards.map((board, idx) => (
                    <div
                      key={idx}
                      className={`p-2 rounded cursor-pointer hover:bg-gray-700 ${
                        selectedBoard?.port === board.port ? 'bg-purple-600' : ''
                      }`}
                      onClick={() => {
                        setSelectedBoard(board);
                        setShowBoardDropdown(false);
                      }}
                    >
                      <div className="text-sm font-medium text-white">{board.boardType}</div>
                      <div className="text-xs text-gray-400">{board.port}</div>
                    </div>
                  ))
                )}
                <Button
                  size="sm"
                  variant="ghost"
                  className="w-full mt-2 text-xs text-purple-400 hover:bg-gray-700"
                  onClick={() => {
                    detectBoards();
                    setShowBoardDropdown(false);
                  }}
                >
                  Refresh Boards
                </Button>
              </div>
            </div>
          )}
        </div>
        
        {/* Upload Button */}
        <Button
          size="sm"
          className={`${isUploading ? 'bg-yellow-600 hover:bg-yellow-700' : 'bg-purple-600 hover:bg-purple-700'} text-white font-medium`}
          onClick={handleUpload}
          disabled={!selectedBoard || isUploading}
        >
          <Upload className="h-4 w-4 mr-1" />
          {isUploading ? 'Uploading...' : 'Upload'}
        </Button>
        
        <img src={logoImage} alt="Logo" className="h-8 w-8" />
      </div>
    </div>
  );
}