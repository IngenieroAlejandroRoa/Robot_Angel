import * as React from "react";
import { Input } from "./ui/input";
import { Button } from "./ui/button";
import { useEffect, useRef, useState, forwardRef, useImperativeHandle } from "react";
import { Terminal as TerminalIcon, ChevronDown } from "lucide-react";

export const Terminal = forwardRef((props, ref) => {
  const [command, setCommand] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const [history, setHistory] = useState([
    { type: "system", content: "RoboIDE Terminal v1.0.0" },
    { type: "system", content: "Robot Control System Ready - Real Terminal" },
    { type: "system", content: "Type commands to execute on the system" },
  ]);
  const [isExecuting, setIsExecuting] = useState(false);
  const [currentDir, setCurrentDir] = useState("");

  const scrollRef = useRef<HTMLDivElement | null>(null);

  // Expose executeCommand method to parent
  useImperativeHandle(ref, () => ({
    executeCommand: async (cmd: string) => {
      return await executeCommandInternal(cmd);
    }
  }));

  // Get initial working directory
  useEffect(() => {
    const terminalBackend = (window as any).angelTerminalBackend;
    if (terminalBackend) {
      terminalBackend.getWorkingDirectory().then((dir: string) => {
        setCurrentDir(dir);
        setHistory(prev => [...prev, { type: "system", content: `Working directory: ${dir}` }]);
      });
    }
  }, []);

  // Auto scroll effect
  useEffect(() => {
    const el = scrollRef.current;
    if (!el || !autoScroll) return;
    el.scrollTo({ top: el.scrollHeight, behavior: "smooth" });
  }, [history, autoScroll]);

  const executeCommandInternal = async (cmd: string) => {
    setIsExecuting(true);
    
    // Add command to history
    setHistory((prev) => [...prev, { type: "input", content: cmd }]);
    
    try {
      const terminalBackend = (window as any).angelTerminalBackend;
      
      if (!terminalBackend) {
        setHistory((prev) => [...prev, { 
          type: "error", 
          content: "Terminal backend not available" 
        }]);
        setIsExecuting(false);
        return;
      }

      // Execute command
      const result = await terminalBackend.executeCommand(cmd, currentDir);
      
      // Update working directory
      const newDir = await terminalBackend.getWorkingDirectory();
      if (newDir !== currentDir) {
        setCurrentDir(newDir);
      }
      
      // Show output
      if (result.output) {
        const outputLines = result.output.trim().split('\n');
        setHistory((prev) => [
          ...prev,
          ...outputLines.map((line: string) => ({ type: "output" as const, content: line }))
        ]);
      }
      
      // Show errors
      if (result.error) {
        const errorLines = result.error.trim().split('\n');
        setHistory((prev) => [
          ...prev,
          ...errorLines.map((line: string) => ({ type: "error" as const, content: line }))
        ]);
      }
      
      // Show exit code if non-zero
      if (result.exitCode !== 0 && !result.output && !result.error) {
        setHistory((prev) => [...prev, { 
          type: "error", 
          content: `Command exited with code ${result.exitCode}` 
        }]);
      }
      
    } catch (error) {
      setHistory((prev) => [...prev, { 
        type: "error", 
        content: `Error: ${error}` 
      }]);
    } finally {
      setIsExecuting(false);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (command.trim() && !isExecuting) {
      const cmd = command.trim();
      setCommand("");
      await executeCommandInternal(cmd);
    }
  };

  const getLineColor = (type: string) => {
    switch (type) {
      case "system":
        return "text-blue-400";
      case "input":
        return "text-gray-200";
      case "output":
        return "text-gray-300";
      case "error":
        return "text-red-400";
      case "success":
        return "text-green-400";
      default:
        return "text-gray-300";
    }
  };

  const getPrompt = (type: string) => {
    switch (type) {
      case "input":
        return "$ ";
      case "output":
        return "  ";
      case "error":
        return "! ";
      case "success":
        return "✓ ";
      default:
        return "  ";
    }
  };

  return (
    <div className="bg-gray-900 flex flex-col min-h-0 h-full w-full">
      {/* Header */}
      <div className="bg-gray-800 border-b border-gray-700 flex items-center px-4 py-1.5 justify-between">
        <div className="flex items-center flex-1">
          <TerminalIcon className="h-4 w-4 text-purple-400 mr-2" />
          <span className="text-gray-200 text-sm">Terminal</span>
        </div>

        {/* Auto-scroll toggle */}
        <Button
          size="icon"
          variant="ghost"
          title={autoScroll ? "Auto-scroll ON" : "Auto-scroll OFF"}
          onClick={() => setAutoScroll((prev) => !prev)}
          className={`h-8 w-8 p-0 ml-1 ${
            autoScroll
              ? "text-purple-400 hover:text-purple-300"
              : "text-gray-500 hover:text-gray-300"
          }`}
        >
          <ChevronDown className="h-4 w-4" />
        </Button>
      </div>

      {/* Terminal output area */}
      <div
        ref={scrollRef}
        className="flex-1 min-h-0 p-4 overflow-auto z-0 font-mono text-sm space-y-1"
      >
        {history.map((line, index) => (
          <div
            key={index}
            className={`${getLineColor(line.type)} leading-relaxed whitespace-pre-wrap break-all`}
          >
            <span className="text-purple-400 mr-1">{getPrompt(line.type)}</span>
            {line.content}
          </div>
        ))}
        {isExecuting && (
          <div className="text-yellow-400 leading-relaxed">
            <span className="text-purple-400 mr-1">⟳</span>
            Executing command...
          </div>
        )}
      </div>

      {/* Command input */}
      <div className="border-t border-gray-700 p-4 z-10">
        <form onSubmit={handleSubmit} className="flex items-center">
          <span className="text-purple-400 mr-2 font-mono">{currentDir ? currentDir.split('/').pop() || '$' : '$'}</span>
          <Input
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="Enter command..."
            className="bg-gray-800 border-gray-600 text-gray-200 font-mono text-sm"
            autoFocus
            disabled={isExecuting}
          />
        </form>
      </div>
    </div>
  );
});
