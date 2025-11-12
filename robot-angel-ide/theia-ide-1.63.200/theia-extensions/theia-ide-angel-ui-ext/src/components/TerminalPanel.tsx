import * as React from "react";
import { useState, useRef, forwardRef, useImperativeHandle } from "react";
import { Terminal } from "./Terminal";
import { Button } from "./ui/button";
import { Plus, X } from "lucide-react";

interface TerminalInstance {
  id: string;
  ref: React.RefObject<any>;
}

export const TerminalPanel = forwardRef((props, ref) => {
  const [terminals, setTerminals] = useState<TerminalInstance[]>([
    { id: "terminal-0", ref: React.createRef() }
  ]);
  
  // Track the terminal ref where micro-ROS is running
  const microRosTerminalRef = useRef<React.RefObject<any> | null>(null);

  // Expose methods to parent
  useImperativeHandle(ref, () => ({
    executeCommand: async (cmd: string) => {
      if (terminals.length > 0 && terminals[0].ref.current) {
        return await terminals[0].ref.current.executeCommand(cmd);
      }
    },
    executeInNewTerminal: async (cmd: string, label?: string) => {
      // Add new terminal
      const newId = `terminal-${Date.now()}`;
      const newRef = React.createRef<any>();
      
      // Store this ref as the micro-ROS terminal if this is a micro-ROS command
      if (cmd.includes('micro_ros_agent')) {
        microRosTerminalRef.current = newRef;
      }
      
      setTerminals(prev => [
        ...prev,
        { id: newId, ref: newRef }
      ]);
      
      // Wait for terminal to be created and execute command
      setTimeout(async () => {
        if (newRef.current && typeof newRef.current.executeCommand === 'function') {
          await newRef.current.executeCommand(cmd);
        }
      }, 500);
      
      return newRef;
    },
    stopMicroRos: async () => {
      // Send Ctrl+C to the specific terminal where micro-ROS is running
      if (microRosTerminalRef.current && microRosTerminalRef.current.current) {
        try {
          // Find the terminal index
          const terminalIndex = terminals.findIndex(t => t.ref === microRosTerminalRef.current);
          
          if (terminalIndex !== -1 && terminals[terminalIndex].ref.current) {
            // Send Ctrl+C to stop the process
            await terminals[terminalIndex].ref.current.sendInterrupt();
            microRosTerminalRef.current = null;
            return true;
          }
        } catch (error) {
          console.error('Error stopping micro-ROS:', error);
        }
      }
      // If no terminal ref, consider it stopped
      microRosTerminalRef.current = null;
      return true;
    }
  }));

  const addTerminal = () => {
    const newId = `terminal-${Date.now()}`;
    setTerminals(prev => [
      ...prev,
      { id: newId, ref: React.createRef() }
    ]);
  };

  const removeTerminal = (id: string) => {
    // Don't remove if it's the last terminal
    if (terminals.length === 1) {
      return;
    }
    
    // If the terminal being removed is the micro-ROS terminal, clear the ref
    const terminalToRemove = terminals.find(t => t.id === id);
    if (terminalToRemove && microRosTerminalRef.current === terminalToRemove.ref) {
      microRosTerminalRef.current = null;
    }
    
    setTerminals(prev => prev.filter(t => t.id !== id));
  };

  return (
    <div className="h-full flex flex-col bg-gray-900 relative">
      {/* Split terminals container - VERTICAL */}
      <div className="flex-1 flex flex-col min-h-0">
        {terminals.map((terminal, index) => (
          <div
            key={terminal.id}
            className="flex-1 min-h-0 relative flex flex-col"
            style={{
              borderBottom: index < terminals.length - 1 ? '1px solid rgb(55, 65, 81)' : 'none'
            }}
          >
            {/* Close button - only show if more than one terminal */}
            {terminals.length > 1 && (
              <div className="absolute top-2 right-10 z-20">
                <Button
                  size="icon"
                  variant="ghost"
                  onClick={() => removeTerminal(terminal.id)}
                  className="h-6 w-6 p-0 bg-gray-800 hover:bg-red-600 text-gray-400 hover:text-white rounded-sm"
                  title="Close terminal"
                >
                  <X className="h-3 w-3" />
                </Button>
              </div>
            )}

            {/* Terminal component */}
            <Terminal ref={terminal.ref} />
          </div>
        ))}
      </div>

      {/* Add terminal button - in the first terminal header */}
      <div className="absolute top-1.5 right-11 z-30">
        <Button
          size="icon"
          variant="ghost"
          onClick={addTerminal}
          className="h-6 w-6 p-0 text-purple-400 hover:text-purple-300 hover:bg-gray-700/50"
          title="Split terminal vertically"
        >
          <Plus className="h-3.5 w-3.5" />
        </Button>
      </div>
    </div>
  );
});
