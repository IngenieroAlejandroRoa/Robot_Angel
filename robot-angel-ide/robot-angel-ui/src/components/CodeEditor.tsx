import { useState, useRef, useEffect } from "react";
import { ScrollArea } from "./ui/scroll-area";

export function CodeEditor() {
  const [cursorPosition, setCursorPosition] = useState({ line: 1, col: 1 });
  const [activeFile, setActiveFile] = useState("main.py");
  const [language] = useState("Python");
  const [status, setStatus] = useState("Saved");

  const codeRef = useRef<HTMLPreElement | null>(null);

  const codeContent = `import numpy as np
from robot_controller import RobotController
import time

# Initialize robot controller
robot = RobotController()

def main():
    """
    Main robotics control loop
    """
    print("Starting robot control system...")
    
    # Set initial position
    robot_position = np.array([0.0, 0.0, 0.0])
    target_position = np.array([10.0, 15.0, 0.0])
    
    # Control parameters
    speed = 2.5
    tolerance = 0.1
    
    # Main control loop
    while True:
        # Get current sensor readings
        sensor_data = robot.get_sensor_data()
        current_pos = robot.get_position()
        
        # Calculate distance to target
        distance = np.linalg.norm(target_position - current_pos)
        
        if distance < tolerance:
            print("Target reached!")
            robot.stop()
            break
        
        # Calculate movement vector
        direction = (target_position - current_pos) / distance
        velocity = direction * speed
        
        # Send commands to robot
        robot.set_velocity(velocity)
        
        # Log status
        print(f"Position: {current_pos}, Distance: {distance:.2f}")
        
        time.sleep(0.1)

if __name__ == "__main__":
    main()`;

  // Detect line and column on click
  const handleClick = (e: React.MouseEvent) => {
    const selection = window.getSelection();
    if (!selection || !codeRef.current) return;

    const range = selection.getRangeAt(0);
    const pre = codeRef.current;
    const textBeforeCursor = pre.innerText.slice(0, range.startOffset);
    const lines = textBeforeCursor.split("\n");
    const line = lines.length;
    const col = lines[lines.length - 1].length + 1;

    setCursorPosition({ line, col });
  };

  // Fake autosave simulation
  useEffect(() => {
    const timer = setTimeout(() => setStatus("Saved"), 1000);
    return () => clearTimeout(timer);
  }, [cursorPosition]);

  return (
    <div className="flex flex-col flex-1 bg-gray-900 border-r border-gray-700 min-h-0 overflow-hidden">
      {/* File tabs */}
      <div className="bg-gray-800 border-b border-gray-700 flex items-center px-4 py-2 z-10">
        <div
          onClick={() => setActiveFile("main.py")}
          className={`px-3 py-1 rounded-t text-sm mr-2 cursor-pointer ${
            activeFile === "main.py"
              ? "bg-purple-600 text-white"
              : "bg-gray-700 text-gray-300 hover:bg-gray-600"
          }`}
        >
          main.py
        </div>
        <div
          onClick={() => setActiveFile("robot_controller.py")}
          className={`px-3 py-1 rounded-t text-sm cursor-pointer ${
            activeFile === "robot_controller.py"
              ? "bg-purple-600 text-white"
              : "bg-gray-700 text-gray-300 hover:bg-gray-600"
          }`}
        >
          robot_controller.py
        </div>
      </div>

      {/* Scrollable code area */}
      <ScrollArea className="flex-1 h-full overflow-auto">
        <div className="p-4 font-mono text-sm w-max min-w-full">
          <pre
            ref={codeRef}
            onClick={handleClick}
            className="text-gray-200 leading-relaxed select-text cursor-text"
          >
            <code>
              {codeContent.split("\n").map((line, index) => (
                <div key={index} className="flex">
                  <span className="w-12 text-right pr-4 text-gray-500 select-none border-r border-gray-700 mr-4">
                    {index + 1}
                  </span>
                  <span className="flex-1 whitespace-pre">
                    {line.split("").map((char, charIndex) => {
                      // Comentarios
                      if (line.trim().startsWith("#")) {
                        return (
                          <span key={charIndex} className="text-green-400">
                            {char}
                          </span>
                        );
                      }
                      // Palabras clave
                      if (
                        line.includes("def ") ||
                        line.includes("class ") ||
                        line.includes("import ") ||
                        line.includes("from ")
                      ) {
                        if (
                          char === " " &&
                          (line.includes("def ") ||
                            line.includes("class ") ||
                            line.includes("import ") ||
                            line.includes("from "))
                        ) {
                          return (
                            <span key={charIndex} className="text-purple-400">
                              {char}
                            </span>
                          );
                        }
                      }
                      // Cadenas
                      if (['"', "'"].includes(char)) {
                        return (
                          <span key={charIndex} className="text-yellow-400">
                            {char}
                          </span>
                        );
                      }
                      // Paréntesis
                      if (["(", ")", "[", "]", "{", "}"].includes(char)) {
                        return (
                          <span key={charIndex} className="text-gray-300">
                            {char}
                          </span>
                        );
                      }
                      return <span key={charIndex}>{char}</span>;
                    })}
                  </span>
                </div>
              ))}
            </code>
          </pre>
        </div>
      </ScrollArea>

      {/* Status bar */}
      <div className="bg-gray-800 border-t border-gray-700 text-gray-300 text-xs px-4 py-2 flex items-center justify-between font-mono">
        <div className="flex items-center gap-4">
          <span>{activeFile}</span>
          <span>| {language}</span>
          <span>| Ln {cursorPosition.line}, Col {cursorPosition.col}</span>
        </div>
        <div
          className={`${
            status === "Saved" ? "text-green-400" : "text-yellow-400"
          }`}
        >
          {status}
        </div>
      </div>
    </div>
  );
}
