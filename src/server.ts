import { McpServer } from "@modelcontextprotocol/sdk/server/mcp.js";
import { registerResources } from "./resources/registry.js";
import { registerTools } from "./tools/registry.js";
import { registerPrompts } from "./prompts/registry.js";

export function createServer(): McpServer {
  const server = new McpServer({
    name: "ftc-mcp",
    version: "1.0.0",
  });

  registerResources(server);
  registerTools(server);
  registerPrompts(server);

  return server;
}
