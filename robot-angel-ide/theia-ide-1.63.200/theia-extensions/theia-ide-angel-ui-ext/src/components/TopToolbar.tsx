import * as React from "react";
import { Button } from "./ui/button";
import { Separator } from "./ui/separator";
import { Play, Square, Bug, Settings, FileText, FolderOpen, Save, Zap, Cpu } from "lucide-react";
import logoImage from './assets/0619f22ea1d44864927116629fbe5ab4c93093fe.png';

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
  isRunning
}: TopToolbarProps) {
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
          className="bg-purple-600 hover:bg-purple-700 text-white"
        >
          <Cpu className="h-4 w-4 mr-1" />
          Micro-ROS
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
        <div className="bg-purple-600 px-3 py-1 rounded text-white font-medium">
          Upload
        </div>
        <img src={logoImage} alt="Logo" className="h-8 w-8" />
      </div>
    </div>
  );
}