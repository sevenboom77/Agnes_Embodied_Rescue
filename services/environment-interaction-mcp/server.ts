import { Server } from "../robot-mcp/node_modules/@modelcontextprotocol/sdk/dist/esm/server/index.js";
import { StdioServerTransport } from "../robot-mcp/node_modules/@modelcontextprotocol/sdk/dist/esm/server/stdio.js";
import { CallToolRequestSchema, ErrorCode, ListToolsRequestSchema, McpError } from "../robot-mcp/node_modules/@modelcontextprotocol/sdk/dist/esm/types.js";
import { z } from "../robot-mcp/node_modules/zod/index.js";
import { appendFileSync, readFileSync, statSync } from "node:fs";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { JsonlBridge } from "../robot-mcp/src/transport/jsonl-bridge.ts";

const exec = promisify(execFile);
const ROOT = "/data-ssd/zhongchengkai/agnes";
const GZ = `${ROOT}/runtime/ros2-jazzy/.pixi/envs/default/bin/gz`;
const PYTHON = `${ROOT}/runtime/ros2-jazzy/.pixi/envs/default/bin/python`;
const registryPath = process.env.ENV_INTERACTION_REGISTRY || (process.env.IMPORTED_MAP_DIR ? `${process.env.IMPORTED_MAP_DIR}/interaction-registry.json` : "");
const configuredWorld = process.env.SCENARIO_WORLD_NAME || "imported_building";
const bridge = new JsonlBridge("/bin/bash", [`${ROOT}/services/robot-mcp/ros2_bridge/launch_bridge.sh`]);
const observed = new Map<string, { at: string; pose: any; evidence: any }>();
const trace = (event: any) => { if (process.env.ROBOT_TRACE_DIR) appendFileSync(`${process.env.ROBOT_TRACE_DIR}/environment-events.jsonl`, JSON.stringify({ time: Date.now(), ...event }) + "\n"); };
function doors() { try { return JSON.parse(readFileSync(registryPath, "utf8")).doors || []; } catch { return []; } }
function door(id: string) { return doors().find((x: any) => x.id === id); }
function distance(a: any, b: any) { return Math.hypot(Number(a.x) - Number(b.x), Number(a.y) - Number(b.y)); }
function doorState(d: any, panel: any) {
  const nominal = Number(d.nominal_orientation ?? d.orientation ?? 0);
  return Number.isFinite(panel?.yaw_rad) && Math.abs(panel.yaw_rad - nominal) >= 0.45 ? "OPEN" : "CLOSED";
}
async function inspectDoor(id: string) {
  const d = door(id); if (!d) return { status: "FAILED", code: "UNKNOWN_DOOR", door_id: id };
  const state = await bridge.request("get_robot_state", {}, 10000);
  // Camera frames and odometry arrive on independent ROS callbacks.  Take a
  // short bounded series of fresh observations so a frame captured just before
  // the navigation goal settled cannot hide a physically visible door.  Every
  // candidate still has to come from the real Camera detector.
  let observation: any = null;
  for (let attempt = 0; attempt < 10; attempt += 1) {
    observation = await bridge.request("get_observation", {}, 65000);
    if (observation.camera?.available && (observation.semantic_objects || []).some((item: any) => item.label === "door")) break;
    await new Promise(resolve => setTimeout(resolve, 180));
  }
  const pose = state.pose || observation.robot_pose;
  let visual = (observation.semantic_objects || []).filter((x: any) => x.label === "door");
  let capturedEvidence: any = null;
  if (!visual.length && observation.camera?.available) {
    capturedEvidence = await capturedDoorEvidence(observation.timestamp);
    visual = (capturedEvidence.detections || []).filter((x: any) => x.label === "door");
  }
  const near = d.interaction_pose ? distance(pose, d.interaction_pose) <= 1.6 : distance(pose, d.at) <= 1.6;
  const panel = await linkPose(id);
  const runtimeState = doorState(d, panel);
  const result = { status: near && visual.length ? "SUCCEEDED" : "FAILED", door_id: id, runtime_state: near && visual.length ? runtimeState : "UNKNOWN", robot_pose: pose, interaction_pose: d.interaction_pose || null, camera_observation: visual, evidence: { source: "real Camera observation + current_cv on real frame + ROS2 pose + Gazebo panel pose", frame_hashes: visual.map((x: any) => x.evidence?.frame_sha256).filter(Boolean), observed_at: new Date().toISOString(), near_interaction_pose: near, captured_frame: capturedEvidence, panel_pose: panel } , map_prior: { exists: true, connects: d.connects } };
  if (result.status === "SUCCEEDED") observed.set(id, { at: result.evidence.observed_at, pose, evidence: result });
  trace({ tool: "inspect_door", input: { door_id: id }, result }); return result;
}
async function jointState(id: string) {
  // The managed MCP process can outlive a scenario activation.  Resolve the
  // live Gazebo world from the project-local transport instead of trusting a
  // stale launch environment; this is observation only and never changes the
  // world or the interaction result.
  let world = configuredWorld;
  try {
    const topics = await exec(GZ, ["topic", "-l"], { timeout: 5000 });
    const names = `${topics.stdout}\n${topics.stderr}`.split(/\r?\n/);
    const live = names.map(line => line.match(/^\/world\/([^/]+)\/state$/)?.[1]).find(Boolean);
    if (live) world = live;
  } catch { /* retain configured world for a bounded fallback */ }
  const topic = `/world/${world}/model/door_${id}/joint/${id}_hinge/state`;
  try { const r = await exec(GZ, ["topic", "-e", "-t", topic, "-n", "1"], { timeout: 5000 }); const text = `${r.stdout}\n${r.stderr}`; const match = text.match(/(?:position|angle|data):\s*([-+0-9.eE]+)/); return { topic, raw: text.slice(-2000), angle_rad: match ? Number(match[1]) : null }; } catch (error: any) { return { topic, raw: String(error?.stdout || error?.message || error), angle_rad: null }; }
}
async function modelPose(id: string) {
  try {
    const r = await exec(GZ, ["model", "-m", `door_${id}`, "-p"], { timeout: 5000 });
    const text = `${r.stdout}\n${r.stderr}`;
    const matches = [...text.matchAll(/\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]\s*$/gm)];
    const last = matches.at(-1);
    return { raw: text.slice(-2000), yaw_rad: last ? Number(last[3]) : null };
  } catch (error: any) { return { raw: String(error?.stdout || error?.message || error), yaw_rad: null }; }
}
async function linkPose(id: string) {
  try {
    const r = await exec(GZ, ["model", "-m", `door_${id}`, "-l", "panel"], { timeout: 5000 });
    const text = `${r.stdout}\n${r.stderr}`;
    const matches = [...text.matchAll(/\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]\s*$/gm)];
    const last = matches.at(-1);
    return { raw: text.slice(-2000), yaw_rad: last ? Number(last[3]) : null };
  } catch (error: any) { return { raw: String(error?.stdout || error?.message || error), yaw_rad: null }; }
}
async function capturedDoorEvidence(timestamp: any) {
  try {
    let last: any = { detections: [], source: "real /camera/image_raw bounded fresh-frame series" };
    for (let attempt = 0; attempt < 4; attempt += 1) {
      const frame = await bridge.request("capture_frame", { name: "camera-frame-door-inspect.png" }, 6000);
      const imagePath = `${ROOT}/artifacts/phase10/camera-frame-door-inspect.png`;
      const result = await exec(PYTHON, [`${ROOT}/services/environment-interaction-mcp/door-vision.py`, imagePath, JSON.stringify(frame.timestamp || timestamp)], { timeout: 10000 });
      const own = JSON.parse(result.stdout);
      last = { ...own, source: "real /camera/image_raw capture through interaction bridge", frame_attempt: attempt + 1 };
      if (own.detections?.some((item: any) => item.label === "door")) return last;
      // The live adapter is another real subscriber to the same ROS image
      // topic. Use it only as a bounded fresh-frame fallback.
      const livePath = `${ROOT}/artifacts/interactive-live/camera/latest.png`;
      const liveMetaPath = `${ROOT}/artifacts/interactive-live/camera/latest-meta.json`;
      const ageMs = Date.now() - statSync(livePath).mtimeMs;
      if (ageMs <= 1500) {
        const meta = JSON.parse(readFileSync(liveMetaPath, "utf8"));
        const live = await exec(PYTHON, [`${ROOT}/services/environment-interaction-mcp/door-vision.py`, livePath, JSON.stringify(meta.timestamp || timestamp)], { timeout: 10000 });
        const liveResult = { ...JSON.parse(live.stdout), source: "real /camera/image_raw live adapter frame", freshness_ms: ageMs, live_pose: meta.pose, frame_attempt: attempt + 1 };
        if (liveResult.detections?.some((item: any) => item.label === "door")) return liveResult;
        last = liveResult;
      }
      await new Promise(resolve => setTimeout(resolve, 200));
    }
    return last;
  } catch (error: any) { return { detections: [], error: String(error?.message || error) }; }
}
async function openDoor(id: string) {
  const prior = observed.get(id); const d = door(id); if (!d) return { status: "FAILED", code: "UNKNOWN_DOOR", door_id: id };
  if (!prior) return { status: "FAILED", code: "DOOR_NOT_VISUALLY_OBSERVED", door_id: id, runtime_state: "UNKNOWN" };
  if (prior.evidence?.runtime_state === "OPEN") {
    const result = { status: "SUCCEEDED", code: "ALREADY_OPEN", door_id: id, runtime_state: "OPEN", command: null, verification: { source: "prior real Camera + Gazebo panel pose observation", observed_at: new Date().toISOString() }, no_fake_world_write: true };
    trace({ tool: "open_door", input: { door_id: id }, result }); return result;
  }
  const joint = `${id}_hinge`; let command: any = null;
  const beforePose = await modelPose(id);
  const beforeLinkPose = await linkPose(id);
  const velocityTopic = `/model/door_${id}/joint/${joint}/cmd_vel`;
  try {
    // Gazebo's official JointController exposes a velocity command topic.  A
    // bounded positive pulse is the physical interaction; the joint limit
    // stops the panel and the second command removes residual motion.
    command = await exec(GZ, ["topic", "-t", velocityTopic, "-m", "gz.msgs.Double", "-p", "data: 1.0"], { timeout: 6000 });
    await new Promise(resolve => setTimeout(resolve, 1400));
    await exec(GZ, ["topic", "-t", velocityTopic, "-m", "gz.msgs.Double", "-p", "data: 0.0"], { timeout: 6000 });
  } catch (error: any) { command = { stdout: error?.stdout || "", stderr: error?.stderr || String(error), code: error?.code || 1 }; }
  await new Promise(resolve => setTimeout(resolve, 500));
  const verification = await jointState(id);
  const afterPose = await modelPose(id);
  const afterLinkPose = await linkPose(id);
  const after = await bridge.request("get_observation", {}, 65000).catch((error: any) => ({ status: "FAILED", error: String(error?.message || error) }));
  const jointOpened = Number.isFinite(verification.angle_rad) && verification.angle_rad >= 0.45;
  const poseMoved = Number.isFinite(beforeLinkPose.yaw_rad) && Number.isFinite(afterLinkPose.yaw_rad) && Math.abs(afterLinkPose.yaw_rad - beforeLinkPose.yaw_rad) >= 0.45;
  const moved = jointOpened || poseMoved;
  const result = { status: moved ? "SUCCEEDED" : "FAILED", code: moved ? "DOOR_OPEN_VERIFIED" : "DOOR_JAMMED_OR_UNAVAILABLE", door_id: id, runtime_state: moved ? "OPEN" : "UNKNOWN", command: { topic: velocityTopic, pulse_rad_s: 1.0, pulse_ms: 1400, returncode: command?.code || 0, stdout: String(command?.stdout || "").slice(-1000), stderr: String(command?.stderr || "").slice(-1000) }, verification: { joint_state: verification, model_pose_before: beforePose, model_pose_after: afterPose, panel_link_pose_before: beforeLinkPose, panel_link_pose_after: afterLinkPose, fresh_camera_observation: after, source: "real Gazebo joint/model pose + fresh Camera observation", observed_at: new Date().toISOString() }, no_fake_world_write: true };
  if (!moved) result.runtime_state = "UNKNOWN";
  trace({ tool: "open_door", input: { door_id: id }, result }); return result;
}
const server = new Server({ name: "environment-interaction-mcp", version: "0.1.0" }, { capabilities: { tools: {} } });
const empty = z.object({}).strict(); const doorInput = z.object({ door_id: z.string().min(1) }).strict();
server.setRequestHandler(ListToolsRequestSchema, async () => ({ tools: [
  { name: "inspect_door", description: "Inspect a map-prior door using the robot pose and current real Camera observation.", inputSchema: { type: "object", properties: { door_id: { type: "string" } }, required: ["door_id"], additionalProperties: false } },
  { name: "open_door", description: "Physically command and verify a previously observed Gazebo door joint.", inputSchema: { type: "object", properties: { door_id: { type: "string" } }, required: ["door_id"], additionalProperties: false } },
] }));
server.setRequestHandler(CallToolRequestSchema, async request => {
  const name = request.params.name; let input: any;
  try { input = doorInput.parse(request.params.arguments || {}); } catch (error: any) { throw new McpError(ErrorCode.InvalidParams, error.message); }
  const result = name === "inspect_door" ? await inspectDoor(input.door_id) : name === "open_door" ? await openDoor(input.door_id) : null;
  if (!result) throw new McpError(ErrorCode.InvalidParams, `Unknown tool: ${name}`);
  return { content: [{ type: "text", text: JSON.stringify(result) }], structuredContent: result as any, ...(result.status === "FAILED" ? { isError: true } : {}) };
});
process.on("SIGTERM", () => { bridge.close().finally(() => process.exit(0)); });
process.on("SIGINT", () => { bridge.close().finally(() => process.exit(0)); });
await server.connect(new StdioServerTransport());
