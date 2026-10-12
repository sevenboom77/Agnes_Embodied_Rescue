#!/usr/bin/env node
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { JsonlBridge } from '../../services/robot-mcp/src/transport/jsonl-bridge.ts';

const ROOT = path.resolve(import.meta.dirname, '../..');
const OUT = path.resolve(process.env.INTERACTIVE_LIVE_ROOT || path.join(ROOT, 'artifacts/interactive-live'));
const CAMERA_DIR = path.join(OUT, 'camera');
const latest = path.join(CAMERA_DIR, 'latest.png');
const metaPath = path.join(CAMERA_DIR, 'latest-meta.json');
const statsPath = path.join(CAMERA_DIR, 'camera-stats.json');
fs.mkdirSync(CAMERA_DIR, { recursive: true });
const bridge = new JsonlBridge('/bin/bash', [`${ROOT}/services/robot-mcp/ros2_bridge/launch_bridge.sh`], process.env);
const intervalMs = Math.max(200, Number(process.env.INTERACTIVE_CAMERA_INTERVAL_MS || 350));
let stopped = false;
let sequence = 0;
const stats = { topic: '/camera/image_raw', target_interval_ms: intervalMs, frames_received: 0, first_timestamp: null, last_timestamp: null, timestamp_monotonic: true, timestamp_resets: 0, hashes: [], hashes_seen: [], started_at: new Date().toISOString(), real_sensor: true, source: 'real ROS2 bridge capture_frame' };
function save(file, value) { fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n'); }
async function tick() {
  const [pose, frame] = await Promise.all([bridge.request('get_robot_state', {}, 10000), bridge.request('capture_frame', { name: 'camera-frame-room-a.png' }, 10000)]);
  const source = path.join(ROOT, 'artifacts/phase10/camera-frame-room-a.png');
  const bytes = fs.readFileSync(source);
  const temp = `${latest}.tmp-${process.pid}`;
  fs.writeFileSync(temp, bytes);
  fs.renameSync(temp, latest);
  const timestamp = frame.timestamp || pose.timestamp;
  const value = (timestamp?.sec || 0) + (timestamp?.nanosec || 0) / 1e9;
  const hash = crypto.createHash('sha256').update(bytes).digest('hex').slice(0, 32);
  const prior = stats.last_timestamp ? (stats.last_timestamp.sec || 0) + (stats.last_timestamp.nanosec || 0) / 1e9 : null;
  if (prior !== null && value < prior) { stats.timestamp_resets += 1; stats.hashes = []; stats.first_timestamp = timestamp; stats.timestamp_monotonic = true; }
  if (!stats.first_timestamp) stats.first_timestamp = timestamp;
  stats.last_timestamp = timestamp;
  stats.frames_received += 1;
  stats.hashes.push(hash);
  if (stats.hashes.length > 40) stats.hashes.shift();
  if (!stats.hashes_seen.includes(hash)) stats.hashes_seen.push(hash);
  if (stats.hashes_seen.length > 200) stats.hashes_seen.shift();
  const meta = { sequence: ++sequence, frame_id: `live-${sequence}`, timestamp, captured_at: new Date().toISOString(), width: 320, height: 240, encoding: 'rgb8->png', topic: '/camera/image_raw', pose, byte_length: bytes.length, pixel_hash_prefix: hash, real_sensor: true };
  save(metaPath, meta);
  save(statsPath, { ...stats, updated_at: new Date().toISOString(), pixel_hash_changes: new Set(stats.hashes_seen).size });
}
async function shutdown() { if (stopped) return; stopped = true; save(statsPath, { ...stats, stopped_at: new Date().toISOString(), pixel_hash_changes: new Set(stats.hashes_seen).size }); await bridge.close(); process.exit(0); }
process.on('SIGTERM', shutdown); process.on('SIGINT', shutdown);
try {
  while (!stopped) { try { await tick(); } catch (error) { fs.appendFileSync(path.join(CAMERA_DIR, 'adapter-errors.log'), `${new Date().toISOString()} ${error.message}\n`); } await new Promise(resolve => setTimeout(resolve, intervalMs)); }
} finally { await bridge.close(); }
