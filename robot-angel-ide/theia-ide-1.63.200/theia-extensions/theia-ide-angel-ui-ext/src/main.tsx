
import * as React from "react";
import { createRoot } from "react-dom/client";
// Import without the .tsx extension to satisfy the TypeScript compiler when
// allowImportingTsExtensions is not enabled.
import App from "./App";
import "./index.css";

createRoot(document.getElementById("root")!).render(<App />);
  