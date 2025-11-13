import * as React from "react";
import { ScrollArea } from "./ui/scroll-area";
import { Input } from "./ui/input";
import { Button } from "./ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "./ui/select";
import { useEffect, useRef, useState } from "react";
import { Zap, Send, ChevronDown, RefreshCw } from "lucide-react";

export function SerialMonitor() {
  const [message, setMessage] = useState("");
  const [isConnected, setIsConnected] = useState(false);
  const [autoScroll, setAutoScroll] = useState(true);
  const [selectedPort, setSelectedPort] = useState("");
  const [availablePorts, setAvailablePorts] = useState<string[]>([]);
  const [isLoadingPorts, setIsLoadingPorts] = useState(false);
  const [baudRate, setBaudRate] = useState("9600");
  const [serialData, setSerialData] = useState([
    { type: "system", content: "Serial Monitor v1.0.0", timestamp: new Date().toLocaleTimeString() },
  ]);

  const scrollRef = useRef<HTMLDivElement | null>(null);

  
  // Load available serial ports
  const loadSerialPorts = async () => {
    setIsLoadingPorts(true);
    try {
      // @ts-ignore - Access global board manager backend
      const boardManagerBackend = window.boardManagerBackend;
      
      if (boardManagerBackend) {
        const ports = await boardManagerBackend.getSerialPorts();
        setAvailablePorts(ports);
        
        const timestamp = new Date().toLocaleTimeString();
        if (ports.length > 0) {
          setSerialData((prev) => [
            ...prev,
            { type: "system", content: `Available ports: ${ports.join(', ')}`, timestamp },
          ]);
          
          // Auto-select first port if none selected
          if (!selectedPort && ports.length > 0) {
            setSelectedPort(ports[0]);
          }
        } else {
          setSerialData((prev) => [
            ...prev,
            { type: "system", content: "No serial ports found", timestamp },
          ]);
        }
      } else {
        console.error('Board manager backend not available');
        setSerialData((prev) => [
          ...prev,
          { type: "error", content: "Board manager service not available", timestamp: new Date().toLocaleTimeString() },
        ]);
      }
    } catch (error) {
      console.error('Failed to load serial ports:', error);
      setSerialData((prev) => [
        ...prev,
        { type: "error", content: `Error loading ports: ${error}`, timestamp: new Date().toLocaleTimeString() },
      ]);
    } finally {
      setIsLoadingPorts(false);
    }
  };

  // Load ports on component mount
  useEffect(() => {
    loadSerialPorts();
  }, []);

  useEffect(() => {
    const el = scrollRef.current;
    if (!el || !autoScroll) return;
    el.scrollTo({ top: el.scrollHeight, behavior: "smooth" });
  }, [serialData, autoScroll]);

  const handleSendMessage = (e: React.FormEvent) => {
    e.preventDefault();
    if (message.trim() && isConnected) {
      const timestamp = new Date().toLocaleTimeString();
      setSerialData((prev) => [
        ...prev,
        { type: "send", content: message, timestamp },
      ]);
      setMessage("");

      
      setTimeout(() => {
        setSerialData((prev) => [
          ...prev,
          {
            type: "receive",
            content: `ACK: ${message}`,
            timestamp: new Date().toLocaleTimeString(),
          },
        ]);
      }, 200);
    }
  };

  const toggleConnection = () => {
    const timestamp = new Date().toLocaleTimeString();
    setIsConnected(!isConnected);

    const statusMessage = isConnected
      ? { type: "disconnect", content: `Disconnected from ${selectedPort}`, timestamp }
      : { type: "connect", content: `Connected to ${selectedPort} at ${baudRate} baud`, timestamp };

    setSerialData((prev) => [...prev, statusMessage]);
  };

  const getLineColor = (type: string) => {
    switch (type) {
      case "system": return "text-blue-400";
      case "send": return "text-green-400";
      case "receive": return "text-gray-300";
      case "connect": return "text-purple-400";
      case "disconnect": return "text-yellow-400";
      case "error": return "text-red-400";
      default: return "text-gray-300";
    }
  };

  const getPrefix = (type: string) => {
    switch (type) {
      case "send": return "→ ";
      case "receive": return "← ";
      case "connect": return "⚡ ";
      case "disconnect": return "⚠ ";
      case "error": return "✗ ";
      default: return "  ";
    }
  };

  return (
    
    <div className="flex flex-col border-t border-gray-700 h-full min-h-0">
      
      <div className="bg-gray-800 border-b border-gray-700 flex items-center px-4 py-2 z-10">
        <div className="flex items-center px-3 py-1.5 rounded-t bg-gray-800">
          <Zap className="h-4 w-4 text-purple-400 mr-2" />
          <span className="text-gray-200 text-sm">Serial Monitor</span>
        </div>
      </div>

      
      <div className="bg-gray-800 border-b border-gray-700 p-3 z-10">
        <div className="flex items-center gap-3">
          
          <div className="flex items-center gap-2">
            <span className="text-gray-400 text-sm">Port:</span>
            <Select value={selectedPort} onValueChange={setSelectedPort} disabled={isConnected}>
              <SelectTrigger className="w-32 h-8 bg-gray-700 border-gray-600 text-gray-200 text-sm">
                <SelectValue placeholder="Select port" />
              </SelectTrigger>
              <SelectContent className="bg-gray-800 border-gray-600 text-gray-200 z-50 shadow-lg">
                {availablePorts.length === 0 ? (
                  <SelectItem value="no-ports" disabled>No ports found</SelectItem>
                ) : (
                  availablePorts.map((port) => (
                    <SelectItem key={port} value={port}>{port}</SelectItem>
                  ))
                )}
              </SelectContent>
            </Select>
            <Button
              size="icon"
              variant="ghost"
              onClick={loadSerialPorts}
              disabled={isLoadingPorts || isConnected}
              title="Refresh ports"
              className="h-8 w-8 p-0 text-gray-400 hover:text-gray-200"
            >
              <RefreshCw className={`h-4 w-4 ${isLoadingPorts ? 'animate-spin' : ''}`} />
            </Button>
          </div>

          
          <div className="flex items-center gap-2">
            <span className="text-gray-400 text-sm">Baud:</span>
            <Select value={baudRate} onValueChange={setBaudRate}>
              <SelectTrigger className="w-24 h-8 bg-gray-700 border-gray-600 text-gray-200 text-sm">
                <SelectValue />
              </SelectTrigger>
              <SelectContent className="bg-gray-800 border-gray-600 text-gray-200 z-50 shadow-lg">
                <SelectItem value="9600">9600</SelectItem>
                <SelectItem value="19200">19200</SelectItem>
                <SelectItem value="38400">38400</SelectItem>
                <SelectItem value="57600">57600</SelectItem>
                <SelectItem value="115200">115200</SelectItem>
              </SelectContent>
            </Select>
          </div>

          
          <Button
            size="sm"
            onClick={toggleConnection}
            className={`h-8 text-sm px-4 ${
              isConnected
                ? "bg-red-600 hover:bg-red-700 text-white"
                : "bg-green-600 hover:bg-green-700 text-white"
            }`}
          >
            {isConnected ? "Disconnect" : "Connect"}
          </Button>

          
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

          
          <div className="flex items-center gap-2 ml-auto">
            <div
              className={`w-2 h-2 rounded-full ${
                isConnected ? "bg-green-400" : "bg-gray-500"
              }`}
            ></div>
            <span className="text-gray-400 text-sm">
              {isConnected ? `Connected to ${selectedPort}` : "Disconnected"}
            </span>
          </div>
        </div>
      </div>

      
      <div
        ref={scrollRef}
        className="flex-1 p-4 bg-gray-900 min-h-0 z-0 overflow-auto font-mono text-sm space-y-1"
      >
        {serialData.map((line, index) => (
          <div
            key={index}
            className={`${getLineColor(line.type)} leading-relaxed flex`}
          >
            <span className="text-gray-500 text-xs w-20 mr-2 shrink-0">
              {line.timestamp}
            </span>
            <span className="text-purple-400 mr-1">{getPrefix(line.type)}</span>
            <span className="flex-1">{line.content}</span>
          </div>
        ))}
      </div>

      
      <div className="bg-gray-800 border-t border-gray-700 p-4 z-10">
        <form onSubmit={handleSendMessage} className="flex items-center gap-2">
          <Input
            value={message}
            onChange={(e) => setMessage(e.target.value)}
            placeholder={isConnected ? "Send command..." : "Not connected"}
            disabled={!isConnected}
            className="bg-gray-700 border-gray-600 text-gray-200 font-mono text-sm"
          />
          <Button
            type="submit"
            size="sm"
            disabled={!isConnected || !message.trim()}
            className="bg-purple-600 hover:bg-purple-700 h-8 w-8 p-0"
          >
            <Send className="h-4 w-4" />
          </Button>
        </form>
      </div>
    </div>
  );
}
