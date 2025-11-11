import * as React from "react";
import { useState, useRef, useEffect, forwardRef, useImperativeHandle } from "react";
import Editor from "@monaco-editor/react";

interface CodeEditorProps {
  content: string;
  filePath: string | null;
  onContentChange: (content: string) => void;
}

export const CodeEditor = forwardRef((props: CodeEditorProps, ref) => {
  const { content, filePath, onContentChange } = props;
  const [cursorPosition, setCursorPosition] = useState({ line: 1, col: 1 });
  const [activeFile, setActiveFile] = useState("main.py");
  const [language, setLanguage] = useState("python");
  const [status, setStatus] = useState("Saved");
  const [localContent, setLocalContent] = useState(content);

  const editorRef = useRef<any>(null);

  useImperativeHandle(ref, () => ({
    getContent: () => localContent,
    setContent: (newContent: string) => setLocalContent(newContent),
  }));

  // Update local content when prop changes
  useEffect(() => {
    setLocalContent(content);
    setStatus("Saved");
  }, [content]);

  // Update file info when path changes
  useEffect(() => {
    if (filePath) {
      const fileName = filePath.split('/').pop() || filePath.split('\\').pop() || 'untitled';
      setActiveFile(fileName);
      
      // Detect language from file extension
      const ext = fileName.split('.').pop()?.toLowerCase();
      const langMap: { [key: string]: string } = {
        'py': 'python',
        'js': 'javascript',
        'ts': 'typescript',
        'cpp': 'cpp',
        'c': 'c',
        'h': 'cpp',
        'hpp': 'cpp',
        'java': 'java',
        'json': 'json',
        'xml': 'xml',
        'html': 'html',
        'css': 'css',
        'md': 'markdown',
        'txt': 'plaintext',
      };
      setLanguage(langMap[ext || ''] || 'plaintext');
    } else {
      setActiveFile('untitled.py');
      setLanguage('python');
    }
  }, [filePath]);

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
      setLocalContent(value);
      onContentChange(value);
      setStatus("Modified");
    }
  };

  return (
    <div className="flex flex-col flex-1 bg-gray-900 border-r border-gray-700 min-h-0 overflow-hidden">
      {/* File tabs */}
      <div className="bg-gray-800 border-b border-gray-700 flex items-center px-4 py-2 z-10">
        <div
          className="px-3 py-1 rounded-t text-sm mr-2 bg-purple-600 text-white"
        >
          {activeFile}
        </div>
      </div>

      {/* Monaco Editor */}
      <div className="flex-1 min-h-0">
        <Editor
          height="100%"
          defaultLanguage={language}
          language={language}
          value={localContent}
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
          <span>{filePath || activeFile}</span>
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
});
