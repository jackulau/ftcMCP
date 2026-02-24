/**
 * MCP Resource Registration — Template-Based
 *
 * Inspired by Cloudflare's progressive discovery pattern: instead of registering
 * 70+ individual resources (each adding to context window overhead), we register
 * 9 category-level resource templates. Each template provides a list callback for
 * resource discovery and a read callback for content retrieval.
 *
 * This reduces the resource listing from ~70 entries to 9 templates, saving
 * significant tokens in the MCP protocol's resource listing.
 */

import { McpServer, ResourceTemplate } from "@modelcontextprotocol/sdk/server/mcp.js";
import { CATEGORIES, getResource, listResources } from "../knowledge/index.js";

export function registerResources(server: McpServer): void {
  for (const [category, catDef] of Object.entries(CATEGORIES)) {
    server.resource(
      catDef.label,
      new ResourceTemplate(`ftc://${category}/{topic}`, {
        list: async () => ({
          resources: listResources(category).map(r => ({
            uri: r.uri,
            name: r.name,
            mimeType: "text/plain" as const,
          })),
        }),
      }),
      async (uri, { topic }) => {
        const topicStr = Array.isArray(topic) ? topic[0] : topic;
        const content = getResource(category, topicStr);
        if (content) {
          return {
            contents: [{
              uri: uri.href,
              mimeType: "text/plain",
              text: content,
            }],
          };
        }

        // Topic not found — return available topics for this category
        const available = listResources(category);
        const topicList = available.map(r => `  - ${r.uri} (${r.name})`).join("\n");
        return {
          contents: [{
            uri: uri.href,
            mimeType: "text/plain",
            text: `Topic "${topicStr}" not found in ${category}.\n\nAvailable:\n${topicList}`,
          }],
        };
      }
    );
  }
}
