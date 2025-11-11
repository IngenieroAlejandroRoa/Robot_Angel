import * as React from "react";
import { Input } from "./ui/input";
import { Button } from "./ui/button";
import { useEffect, useRef, useState } from "react";
import { Terminal as TerminalIcon, ChevronDown } from "lucide-react";

export function Terminal() {
  const [command, setCommand] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const [history, setHistory] = useState([
    { type: "system", content: "RoboIDE Terminal v1.0.0" },
    { type: "system", content: "Robot Control System Ready" },
    { type: "system", content: "Type 'help' for available commands" },
  ]);

  const scrollRef = useRef<HTMLDivElement | null>(null);

  // Auto scroll effect
  useEffect(() => {
    const el = scrollRef.current;
    if (!el || !autoScroll) return;
    el.scrollTo({ top: el.scrollHeight, behavior: "smooth" });
  }, [history, autoScroll]);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (command.trim()) {
      setHistory((prev) => [...prev, { type: "input", content: command }]);
      setCommand("");
      
      // Simulate command execution
      setTimeout(() => {
        setHistory((prev) => [
          ...prev,
          { type: "output", content: `Executing: ${command}` },
          { type: "output", content: "Command processed" },
        ]);
      }, 150);
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
        <div className="flex items-center">
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
            className={`${getLineColor(line.type)} leading-relaxed whitespace-pre-wrap`}
          >
            <span className="text-purple-400 mr-1">{getPrompt(line.type)}</span>
            {line.content}
          </div>
        ))}
      </div>

      {/* Command input */}
      <div className="border-t border-gray-700 p-4 z-10">
        <form onSubmit={handleSubmit} className="flex items-center">
          <span className="text-purple-400 mr-2 font-mono">$</span>
          <Input
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="Enter command..."
            className="bg-gray-800 border-gray-600 text-gray-200 font-mono text-sm"
            autoFocus
          />
        </form>
      </div>
    </div>
  );
}
