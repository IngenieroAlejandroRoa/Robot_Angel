import * as React from "react";
import { useState, useRef, useEffect, forwardRef, useImperativeHandle } from "react";
import Editor from "@monaco-editor/react";
import { X } from "lucide-react";

interface FileTab {
  id: string;
  filePath: string;
  content: string;
  isModified: boolean;
}

interface CodeEditorProps {
  content: string;
  filePath: string | null;
  onContentChange: (content: string) => void;
  openFiles: FileTab[];
  activeFileId: string | null;
  onTabClick: (fileId: string) => void;
  onCloseTab: (fileId: string) => void;
}

export const CodeEditor = forwardRef((props: CodeEditorProps, ref) => {
  const { content, filePath, onContentChange, openFiles, activeFileId, onTabClick, onCloseTab } = props;
  const [cursorPosition, setCursorPosition] = useState({ line: 1, col: 1 });
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
      setLanguage('python');
    }
  }, [filePath]);

  const handleEditorDidMount = (editor: any, monaco: any) => {
    editorRef.current = editor;
    
    // Define custom theme matching the UI colors
    monaco.editor.defineTheme('angel-purple', {
      base: 'vs-dark',
      inherit: true,
      rules: [
        { token: 'comment', foreground: '9ca3af', fontStyle: 'italic' },
        { token: 'keyword', foreground: 'c084fc' },
        { token: 'string', foreground: '34d399' },
        { token: 'number', foreground: 'a78bfa' },
        { token: 'type', foreground: '60a5fa' },
        { token: 'class', foreground: '60a5fa' },
        { token: 'function', foreground: 'c084fc' },
        { token: 'variable', foreground: 'd1d5db' },
        { token: 'constant', foreground: 'a78bfa' },
        { token: 'operator', foreground: 'c084fc' },
      ],
      colors: {
        'editor.background': '#111827',
        'editor.foreground': '#d1d5db',
        'editor.lineHighlightBackground': '#1f2937',
        'editorCursor.foreground': '#c084fc',
        'editor.selectionBackground': '#374151',
        'editor.inactiveSelectionBackground': '#37415180',
        'editorLineNumber.foreground': '#6b7280',
        'editorLineNumber.activeForeground': '#d1d5db',
        'editorWidget.background': '#1f2937',
        'editorWidget.border': '#374151',
        'editorSuggestWidget.background': '#1f2937',
        'editorSuggestWidget.border': '#374151',
        'editorSuggestWidget.foreground': '#d1d5db',
        'editorSuggestWidget.selectedBackground': '#9333ea',
        'editorSuggestWidget.highlightForeground': '#c084fc',
        'list.hoverBackground': '#374151',
        'list.activeSelectionBackground': '#9333ea',
        'list.inactiveSelectionBackground': '#374151',
        'editorBracketMatch.background': '#37415180',
        'editorBracketMatch.border': '#c084fc',
      }
    });
    
    // Set custom theme as default
    monaco.editor.setTheme('angel-purple');
    
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

  const getFileName = (path: string | null): string => {
    if (!path) return 'untitled';
    return path.split('/').pop() || path.split('\\').pop() || 'untitled';
  };

  const handleCloseTab = (e: React.MouseEvent, fileId: string) => {
    e.stopPropagation();
    onCloseTab(fileId);
  };

  const activeFile = openFiles.find(f => f.id === activeFileId);

  return (
    <div className="flex flex-col flex-1 bg-gray-900 border-r border-gray-700 min-h-0 overflow-hidden">
      {/* File tabs */}
      <div className="bg-gray-800 border-b border-gray-700 flex items-center px-2 py-2 z-10 overflow-x-auto">
        {openFiles.length === 0 ? (
          <div className="px-3 py-1 text-sm text-gray-500">No files open</div>
        ) : (
          openFiles.map(file => {
            const fileName = getFileName(file.filePath);
            const isActive = file.id === activeFileId;
            
            return (
              <div
                key={file.id}
                onClick={() => onTabClick(file.id)}
                className={`
                  px-3 py-1 rounded-t text-sm mr-1 cursor-pointer
                  flex items-center gap-2 group
                  transition-colors duration-150
                  ${isActive
                    ? "bg-purple-600 text-white"
                    : "bg-gray-700 text-gray-300 hover:bg-gray-600"
                  }
                `}
              >
                <span>{fileName}{file.isModified ? '*' : ''}</span>
                <button
                  onClick={(e) => handleCloseTab(e, file.id)}
                  className={`
                    w-4 h-4 rounded flex items-center justify-center
                    transition-colors
                    ${isActive
                      ? "hover:bg-purple-700"
                      : "hover:bg-gray-800"
                    }
                  `}
                >
                  <X className="w-3 h-3" />
                </button>
              </div>
            );
          })
        )}
      </div>

      {/* Monaco Editor */}
      <div className="flex-1 min-h-0">
        {openFiles.length === 0 ? (
          <div className="flex items-center justify-center h-full text-gray-500">
            <div className="text-center">
              <p className="text-lg mb-2">No files open</p>
              <p className="text-sm">Open a file or create a new one to start coding</p>
            </div>
          </div>
        ) : (
          <Editor
            height="100%"
            defaultLanguage={language}
            language={language}
            value={localContent}
            onChange={handleEditorChange}
            onMount={handleEditorDidMount}
            theme="angel-purple"
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
              domReadOnly: false,
              readOnly: false,
            }}
          />
        )}
      </div>

      {/* Status bar */}
      <div className="bg-gray-800 border-t border-gray-700 text-gray-300 text-xs px-4 py-2 flex items-center justify-between font-mono">
        <div className="flex items-center gap-4">
          <span>{filePath || 'untitled'}</span>
          <span>| {language.toUpperCase()}</span>
          <span>| Ln {cursorPosition.line}, Col {cursorPosition.col}</span>
        </div>
        <div
          className={`${
            activeFile?.isModified ? "text-yellow-400" : "text-green-400"
          }`}
        >
          {activeFile?.isModified ? "Modified" : "Saved"}
        </div>
      </div>
    </div>
  );
});
