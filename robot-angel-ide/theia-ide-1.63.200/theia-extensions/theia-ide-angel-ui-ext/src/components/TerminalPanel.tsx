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

  // Expose method to execute command in first terminal
  useImperativeHandle(ref, () => ({
    executeCommand: async (cmd: string) => {
      if (terminals.length > 0 && terminals[0].ref.current) {
        return await terminals[0].ref.current.executeCommand(cmd);
      }
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
