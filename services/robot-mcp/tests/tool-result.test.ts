import assert from "node:assert/strict";
import test from "node:test";
import { compactRobotToolResult } from "../src/transport/tool-result.ts";

test("large room inspections keep semantic camera evidence without Harness-sized duplication", () => {
  const detections = Array.from({ length: 8 }, (_, index) => ({
    id: `visual-${index}`, label: index === 3 ? "person" : "toolbox", confidence: 0.91,
    bbox: [1, 2, 80, 90], center: [40, 45], source: "ros2_camera_perception",
    frame_id: "camera_link", timestamp: { sec: index + 10, nanosec: 0 }, location: "Room_X",
    view_pose: { x: 1.2, y: 2.3, yaw: 0.4, frame_id: "map" }, view_bearing: 0.8,
    evidence: { method: "cv_color_contour", demo_marker: true, general_recognition: false,
      frame_timestamp: { sec: index + 10, nanosec: 0 }, frame_sha256: "a".repeat(64),
      image_width: 320, image_height: 240, confidence_components: { color: 0.9, fill: 0.8 } },
  }));
  const full = {
    status: "SUCCEEDED", room: "Room_X", inspected: true, scan_views: 8,
    complete_visibility_claimed: false, detections,
    object_observations: [{ object_id: "person", location: "Room_X", labels: ["person"] }],
    evidence: Array.from({ length: 8 }, (_, index) => ({
      yaw: index * Math.PI / 4, timestamp: { sec: index + 10, nanosec: 0 },
      navigation_status: "IDLE", detections,
      motion: {
        status: "SUCCEEDED", motion: "ROTATION_ONLY",
        start_pose: { x: 1, y: 2, yaw: 0, frame_id: "map" },
        end_pose: { x: 1.01, y: 2, yaw: 0.7, frame_id: "map" },
        xy_drift: 0.01, yaw_error: 0.04,
        samples: Array.from({ length: 40 }, (_, sample) => ({
          at: sample, x: 1 + sample / 4000, y: 2, yaw: sample / 100,
          yaw_error: 0.7 - sample / 100,
        })),
      },
    })),
    confidence_history: detections.map(({ id, confidence }) => ({ id, confidence })),
  };
  assert.ok(Buffer.byteLength(JSON.stringify(full)) > 16 * 1024);
  const sent = compactRobotToolResult("inspect_room", full) as any;
  const encoded = JSON.stringify(sent);
  assert.ok(Buffer.byteLength(encoded) <= 12 * 1024);
  assert.equal(sent.status, "SUCCEEDED");
  assert.equal(sent.room, "Room_X");
  assert.equal(sent.scan_views, 8);
  assert.equal(sent.complete_visibility_claimed, false);
  assert.equal(sent.detections.some((item: any) => item.label === "person" &&
    item.source === "ros2_camera_perception" && item.evidence.frame_timestamp), true);
  assert.equal(sent.evidence[0].detection_count, detections.length);
  assert.equal(sent.evidence[0].detection_ids.includes("visual-3"), true);
  assert.equal(sent.evidence[0].motion.samples, undefined);
  assert.equal(sent.evidence[0].motion.sample_summary.count, 40);
  assert.equal(sent.evidence[0].motion.sample_summary.max_xy_deviation_from_start_m > 0, true);
  assert.equal(JSON.parse(encoded).status, "SUCCEEDED");
});

test("large inspection JSON strings and MCP content envelopes are compacted", () => {
  const full = {
    status: "SUCCEEDED", room: "Room_X", inspected: true, scan_views: 8,
    detections: [{ id: "person-frame", label: "person", source: "ros2_camera_perception",
      frame_id: "camera_link", timestamp: { sec: 1 }, evidence: { method: "cv", frame_timestamp: { sec: 1 }, frame_sha256: "b".repeat(64) } }],
    evidence: Array.from({ length: 8 }, (_, index) => ({
      yaw: index, timestamp: { sec: index }, motion: { status: "SUCCEEDED", start_pose: { x: 0, y: 0 },
        samples: Array.from({ length: 50 }, (_, sample) => ({ at: sample, x: sample, y: sample, yaw_error: sample })) },
      detections: [{ id: `frame-${index}`, label: "person" }],
    })),
    confidence_history: Array.from({ length: 100 }, (_, index) => ({ id: `item-${index}`, confidence: 0.9 })),
  };
  const encoded = JSON.stringify(full);
  const wrapped = { content: [{ type: "text", text: encoded }] };
  for (const input of [encoded, wrapped, { structuredContent: full }]) {
    const sent = compactRobotToolResult("inspect_room", input) as any;
    assert.equal(sent.status, "SUCCEEDED");
    assert.equal(sent.evidence[0].motion.samples, undefined);
    assert.equal(sent.evidence[0].motion.sample_summary.count, 50);
    assert.equal(sent.detections.some((item: any) => item.label === "person"), true);
    assert.ok(Buffer.byteLength(JSON.stringify(sent)) <= 12 * 1024);
  }
});

test("small results and non-inspection failures preserve their original semantics", () => {
  const small = { status: "SUCCEEDED", room: "Room_X", detections: [] };
  assert.equal(compactRobotToolResult("inspect_room", small), small);
  const failure = { status: "FAILED", room: "Room_X", code: "MOTION_CONTROL_BUSY", detail: "x".repeat(20_000) };
  const compactFailure = compactRobotToolResult("inspect_room", failure) as any;
  assert.equal(compactFailure.status, "FAILED");
  assert.equal(compactFailure.code, "MOTION_CONTROL_BUSY");
  assert.equal(compactRobotToolResult("get_robot_state", failure), failure);
});
