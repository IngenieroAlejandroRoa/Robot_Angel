import { useState } from "react";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "./ui/tabs";
import { ScrollArea } from "./ui/scroll-area";
import { Button } from "./ui/button";
import { ChevronRight, ChevronDown, FileText, Folder, Variable, Puzzle } from "lucide-react";

export function SidebarPanel() {
  const [expandedFolders, setExpandedFolders] = useState<Set<string>>(new Set(["src"]));
  
  const toggleFolder = (folder: string) => {
    const newExpanded = new Set(expandedFolders);
    if (newExpanded.has(folder)) {
      newExpanded.delete(folder);
    } else {
      newExpanded.add(folder);
    }
    setExpandedFolders(newExpanded);
  };

  return (
    
    <div className="bg-gray-800 border-r border-gray-700 flex flex-col h-full w-full min-w-0">
      <Tabs defaultValue="explorer" className="flex-1 flex flex-col min-h-0">
        <TabsList className="grid w-full grid-cols-3 bg-gray-900 border-b border-gray-700">
          <TabsTrigger 
            value="explorer" 
            className="data-[state=active]:bg-purple-600 data-[state=active]:text-white"
          >
            Explorer
          </TabsTrigger>
          <TabsTrigger 
            value="variables"
            className="data-[state=active]:bg-purple-600 data-[state=active]:text-white"
          >
            Variables
          </TabsTrigger>
          <TabsTrigger 
            value="plugins"
            className="data-[state=active]:bg-purple-600 data-[state=active]:text-white"
          >
            Plugins
          </TabsTrigger>
        </TabsList>
        
        <TabsContent value="explorer" className="flex-1 p-0">
          <ScrollArea className="h-full">
            <div className="p-4">
              <div className="space-y-1">
                
                <div className="flex items-center py-1">
                  <Button
                    variant="ghost"
                    size="sm"
                    className="h-6 w-6 p-0 text-gray-400 hover:text-white"
                    onClick={() => toggleFolder("src")}
                  >
                    {expandedFolders.has("src") ? 
                      <ChevronDown className="h-4 w-4" /> : 
                      <ChevronRight className="h-4 w-4" />
                    }
                  </Button>
                  <Folder className="h-4 w-4 text-purple-400 mr-2" />
                  <span className="text-gray-200">src</span>
                </div>
                
                {expandedFolders.has("src") && (
                  <div className="ml-6 space-y-1">
                    <div className="flex items-center py-1 text-gray-300 hover:text-white hover:bg-gray-700 rounded px-2 cursor-pointer">
                      <FileText className="h-4 w-4 text-blue-400 mr-2" />
                      <span>main.py</span>
                    </div>
                    <div className="flex items-center py-1 text-gray-300 hover:text-white hover:bg-gray-700 rounded px-2 cursor-pointer">
                      <FileText className="h-4 w-4 text-blue-400 mr-2" />
                      <span>robot_controller.py</span>
                    </div>
                    <div className="flex items-center py-1 text-gray-300 hover:text-white hover:bg-gray-700 rounded px-2 cursor-pointer">
                      <FileText className="h-4 w-4 text-green-400 mr-2" />
                      <span>config.json</span>
                    </div>
                  </div>
                )}
                
                <div className="flex items-center py-1">
                  <Button
                    variant="ghost"
                    size="sm"
                    className="h-6 w-6 p-0 text-gray-400 hover:text-white"
                    onClick={() => toggleFolder("tests")}
                  >
                    {expandedFolders.has("tests") ? 
                      <ChevronDown className="h-4 w-4" /> : 
                      <ChevronRight className="h-4 w-4" />
                    }
                  </Button>
                  <Folder className="h-4 w-4 text-purple-400 mr-2" />
                  <span className="text-gray-200">tests</span>
                </div>
              </div>
            </div>
          </ScrollArea>
        </TabsContent>
        
        <TabsContent value="variables" className="flex-1 p-0">
          <ScrollArea className="h-full">
            <div className="p-4 space-y-2">
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center mb-2">
                  <Variable className="h-4 w-4 text-purple-400 mr-2" />
                  <span className="text-gray-200">robot_position</span>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  array([10.5, 25.3, 0.0])
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center mb-2">
                  <Variable className="h-4 w-4 text-purple-400 mr-2" />
                  <span className="text-gray-200">speed</span>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  2.5 (float)
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center mb-2">
                  <Variable className="h-4 w-4 text-purple-400 mr-2" />
                  <span className="text-gray-200">sensor_data</span>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  dict (5 items)
                </div>
              </div>
            </div>
          </ScrollArea>
        </TabsContent>
        
        <TabsContent value="plugins" className="flex-1 p-0">
          <ScrollArea className="h-full">
            <div className="p-4 space-y-3">
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <Puzzle className="h-4 w-4 text-green-400 mr-2" />
                    <span className="text-gray-200">ROS Integration</span>
                  </div>
                  <div className="w-2 h-2 bg-green-400 rounded-full"></div>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  Active • v2.1.4
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <Puzzle className="h-4 w-4 text-green-400 mr-2" />
                    <span className="text-gray-200">SLAM Toolkit</span>
                  </div>
                  <div className="w-2 h-2 bg-green-400 rounded-full"></div>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  Active • v1.8.2
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <Puzzle className="h-4 w-4 text-yellow-400 mr-2" />
                    <span className="text-gray-200">Vision Processing</span>
                  </div>
                  <div className="w-2 h-2 bg-yellow-400 rounded-full"></div>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  Update Available • v3.0.1
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <Puzzle className="h-4 w-4 text-gray-400 mr-2" />
                    <span className="text-gray-200">Motion Planning</span>
                  </div>
                  <div className="w-2 h-2 bg-gray-500 rounded-full"></div>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  Disabled • v1.5.0
                </div>
              </div>
              
              <div className="bg-gray-900 p-3 rounded border border-gray-600">
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <Puzzle className="h-4 w-4 text-purple-400 mr-2" />
                    <span className="text-gray-200">Simulation Bridge</span>
                  </div>
                  <div className="w-2 h-2 bg-blue-400 rounded-full"></div>
                </div>
                <div className="text-sm text-gray-400 ml-6">
                  Installing... • v2.3.0
                </div>
              </div>
            </div>
          </ScrollArea>
        </TabsContent>
      </Tabs>
    </div>
  );
}