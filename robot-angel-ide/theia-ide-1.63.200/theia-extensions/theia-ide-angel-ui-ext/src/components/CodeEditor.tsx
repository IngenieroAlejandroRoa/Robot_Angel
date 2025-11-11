import * as React from "react";
import { useState, useRef } from "react";
import Editor from "@monaco-editor/react";

export function CodeEditor() {
  const [cursorPosition, setCursorPosition] = useState({ line: 1, col: 1 });
  const [activeFile, setActiveFile] = useState("main.py");
  const [language] = useState("python");
  const [status, setStatus] = useState("Saved");
  const [code, setCode] = useState(`import numpy as np
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
    main()`);

  const editorRef = useRef<any>(null);

  const handleEditorDidMount = (editor: any) => {
    editorRef.current = editor;
    
    // Listen to cursor position changes
    editor.onDidChangeCursorPosition((e: any) => {
      setCursorPosition({
        line: e.position.lineNumber,
        col: e.position.column,
      });
    });

    // Listen to content changes
    editor.onDidChangeModelContent(() => {
      setStatus("Modified");
      setTimeout(() => setStatus("Saved"), 1000);
    });
  };

  const handleEditorChange = (value: string | undefined) => {
    if (value !== undefined) {
      setCode(value);
      setStatus("Modified");
    }
  };

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

      {/* Monaco Editor */}
      <div className="flex-1 min-h-0">
        <Editor
          height="100%"
          defaultLanguage={language}
          language={language}
          value={code}
          onChange={handleEditorChange}
          onMount={handleEditorDidMount}
          theme="vs-dark"
          options={{
            fontSize: 14,
            fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
            minimap: { enabled: true },
            scrollBeyondLastLine: false,
            automaticLayout: true,
            tabSize: 4,
            insertSpaces: true,
            wordWrap: "on",
            lineNumbers: "on",
            renderWhitespace: "selection",
            cursorStyle: "line",
            cursorBlinking: "smooth",
            smoothScrolling: true,
            contextmenu: true,
            folding: true,
            foldingStrategy: "indentation",
            showFoldingControls: "always",
            bracketPairColorization: {
              enabled: true,
            },
          }}
        />
      </div>

      {/* Status bar */}
      <div className="bg-gray-800 border-t border-gray-700 text-gray-300 text-xs px-4 py-2 flex items-center justify-between font-mono">
        <div className="flex items-center gap-4">
          <span>{activeFile}</span>
          <span>| {language.toUpperCase()}</span>
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
