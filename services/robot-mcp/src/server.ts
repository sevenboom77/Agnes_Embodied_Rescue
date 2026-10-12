import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { appendFileSync } from "node:fs";
import {
  CallToolRequestSchema,
  ErrorCode,
  ListToolsRequestSchema,
  McpError,
} from "@modelcontextprotocol/sdk/types.js";
import { z } from "zod";
import { InProcessRobotGateway } from "./transport/in-process-gateway.ts";
import { Ros2RobotGateway } from "./transport/ros2-gateway.ts";
import { compactRobotToolResult } from "./transport/tool-result.ts";
import type { RobotGatewayBackend } from "./transport/robot-gateway.ts";
import {
  createMockRescueScenario,
  type MockScenarioName,
} from "./mock/scenario.ts";

const argumentValue = (name: string): string | undefined => {
  const index = process.argv.indexOf(name);
  return index >= 0 ? process.argv[index + 1] : undefined;
};

const requestedScenario = argumentValue("--scenario") ?? "static";
if (requestedScenario !== "static" && requestedScenario !== "dynamic_rescue") {
  throw new Error(`unknown mock scenario: ${requestedScenario}`);
}
const eventLogPath = argumentValue("--event-log");
const selectedBackend = process.env.ROBOT_GATEWAY ?? "mock";
if (!["mock", "in-process", "ros2"].includes(selectedBackend)) throw new Error("INVALID_ROBOT_GATEWAY");
const robot: RobotGatewayBackend = selectedBackend === "ros2" ? new Ros2RobotGateway() : new InProcessRobotGateway(createMockRescueScenario({
  name: requestedScenario as MockScenarioName,
  onEvent: (event) => {
    if (eventLogPath) {
      appendFileSync(eventLogPath, `${JSON.stringify(event)}\n`, "utf8");
    }
  },
}).robot);

type ToolResult = {
  content: [{ type: "text"; text: string }];
  structuredContent: Record<string, unknown>;
};

type ToolSpec = {
  name: string;
  description: string;
  inputSchema: Record<string, unknown>;
  parse(input: unknown): unknown;
  execute(input: unknown): unknown;
};

const emptySchema = {
  type: "object",
  properties: {},
  additionalProperties: false,
};
const observationSchema = {
  type: "object",
  properties: { object_id: { type: "string" } },
  additionalProperties: false,
};
const locationSchema = {
  type: "object",
  properties: { location: { type: "string" } },
  required: ["location"],
  additionalProperties: false,
};
const objectSchema = {
  type: "object",
  properties: { object_id: { type: "string" } },
  required: ["object_id"],
  additionalProperties: false,
};

const emptyInput = z.object({}).strict();
const observationInput = z.object({ object_id: z.string().optional() }).strict();
const locationInput = z.object({ location: z.string() }).strict();
const objectInput = z.object({ object_id: z.string() }).strict();

const specs: ToolSpec[] = [
  {
    name: "get_robot_state",
    description: "Read robot pose and navigation state from the selected backend.",
    inputSchema: emptySchema,
    parse: (input) => emptyInput.parse(input),
    execute: () => robot.getState(),
  },
  {
    name: "get_world_state",
    description: "Read backend-visible static map, robot state and confirmed runtime facts, never hidden simulator occupants.",
    inputSchema: emptySchema,
    parse: (input) => emptyInput.parse(input),
    execute: () => robot.getWorldState(),
  },
  {
    name: "get_observation",
    description: "Read passive observations at the robot location.",
    inputSchema: observationSchema,
    parse: (input) => observationInput.parse(input),
    execute: (input) => robot.getObservation(input as { object_id?: string }),
  },
  {
    name: "navigate_to",
    description: "Navigate to a registered location through the selected backend; for a task-origin return, the existing location string __MISSION_ORIGIN__:<mission_id> resolves only to that mission's real origin captured after mission_reset.",
    inputSchema: locationSchema,
    parse: (input) => locationInput.parse(input),
    execute: (input) => robot.navigateTo((input as { location: string }).location),
  },
  {
    name: "inspect_room",
    description: "Actively inspect the room occupied by the robot.",
    inputSchema: locationSchema,
    parse: (input) => locationInput.parse(input),
    execute: (input) => robot.inspectRoom((input as { location: string }).location),
  },
  {
    name: "inspect_object",
    description: "Actively inspect one object.",
    inputSchema: objectSchema,
    parse: (input) => objectInput.parse(input),
    execute: (input) => robot.inspectObject((input as { object_id: string }).object_id),
  },
  {
    name: "stop_robot",
    description: "Stop the robot; ROS2 cancels the active goal and publishes zero Twist.",
    inputSchema: emptySchema,
    parse: (input) => emptyInput.parse(input),
    execute: () => robot.stop(),
  },
  {
    name: "list_known_locations",
    description: "List locations known to the mock robot contract.",
    inputSchema: emptySchema,
    parse: (input) => emptyInput.parse(input),
    execute: () => robot.listKnownLocations(),
  },
];

const structured = (value: unknown): ToolResult => ({
  content: [{ type: "text", text: JSON.stringify(value) }],
  structuredContent: Array.isArray(value) ? { items: value } : (value as Record<string, unknown>),
});

const server = new Server(
  { name: "embodied-rescue-robot-mcp", version: "0.1.0" },
  { capabilities: { tools: {} } },
);

server.setRequestHandler(ListToolsRequestSchema, async () => ({
  tools: specs.map(({ name, description, inputSchema }) => ({
    name,
    description,
    inputSchema,
  })),
}));

server.setRequestHandler(CallToolRequestSchema, async (request, extra) => {
  const spec = specs.find((candidate) => candidate.name === request.params.name);
  if (!spec) {
    throw new McpError(ErrorCode.InvalidParams, `Unknown tool: ${request.params.name}`);
  }
  let executionPending = false;
  let removeCancellation: (() => void) | undefined;
  try {
    const input = spec.parse(request.params.arguments ?? {});
    // The MCP SDK sends notifications/cancelled when its request deadline or
    // caller signal fires. Propagate it to whichever real motion owner is
    // active; otherwise a rejected promise can leave a live Nav2 goal or a
    // direct /cmd_vel inspection spinning behind.
    if (["navigate_to", "inspect_room", "inspect_object"].includes(spec.name) && extra?.signal) {
      if (extra.signal.aborted) throw new Error("MCP request canceled before navigation started");
      executionPending = true;
      const onCancellation = () => {
        if (!executionPending) return;
        void Promise.resolve(robot.stop()).then((result) => {
          if (process.env.ROBOT_TRACE_DIR) appendFileSync(`${process.env.ROBOT_TRACE_DIR}/mcp-events.jsonl`,
            `${JSON.stringify({time:Date.now(),event:"mcp_request_cancelled",request_id:(request as any).id??null,tool:spec.name,result})}\n`);
        }).catch((error) => {
          if (process.env.ROBOT_TRACE_DIR) appendFileSync(`${process.env.ROBOT_TRACE_DIR}/mcp-events.jsonl`,
            `${JSON.stringify({time:Date.now(),event:"mcp_request_cancel_failed",request_id:(request as any).id??null,tool:spec.name,error:String(error)})}\n`);
        });
      };
      extra.signal.addEventListener("abort", onCancellation, { once: true });
      removeCancellation = () => extra.signal.removeEventListener("abort", onCancellation);
    }
    const value = await spec.execute(input);
    executionPending = false;
    if (process.env.ROBOT_TRACE_DIR) appendFileSync(`${process.env.ROBOT_TRACE_DIR}/mcp-events.jsonl`,
      JSON.stringify({time:Date.now(),request_id:(request as any).id??null,
        tool:spec.name,input,result:value,backend:selectedBackend})+"\n");
    // Keep the complete bridge result above in the project-owned raw MCP trace;
    // send a bounded semantic equivalent so Harness event capture cannot cut
    // an otherwise successful inspection into invalid JSON.
    const agentValue = compactRobotToolResult(spec.name, value);
    return { ...structured(agentValue), ...(value && typeof value === "object" && (value as any).ok === false ? { isError: true } : {}) };
  } catch (error) {
    if (error instanceof z.ZodError) {
      throw new McpError(ErrorCode.InvalidParams, error.message);
    }
    throw error;
  } finally {
    executionPending = false;
    removeCancellation?.();
  }
});

let closing = false;
async function shutdown() {
  if (closing) return;
  closing = true;
  if (robot instanceof Ros2RobotGateway) await robot.close();
  await server.close();
}
process.on("SIGTERM", () => { void shutdown().finally(() => process.exit(0)); });
process.on("SIGINT", () => { void shutdown().finally(() => process.exit(0)); });
process.stdin.on("end", () => { void shutdown(); });
await server.connect(new StdioServerTransport());
