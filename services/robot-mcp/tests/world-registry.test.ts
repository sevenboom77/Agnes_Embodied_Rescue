import assert from "node:assert/strict";
import test from "node:test";
import { fileURLToPath } from "node:url";
import { WorldRegistry } from "../src/transport/world-registry.ts";
import { RuntimeWorldState } from "../src/transport/runtime-world-state.ts";
import { Ros2RobotGateway } from "../src/transport/ros2-gateway.ts";
import type { JsonlBridge } from "../src/transport/jsonl-bridge.ts";
import { BridgeError } from "../src/transport/jsonl-bridge.ts";

const load=()=>WorldRegistry.load(fileURLToPath(new URL("../../../simulation/phase9f/world_registry.json",import.meta.url)));
test("static registry validates frame, finite poses, adjacency and excludes hidden metadata",()=>{
  const r=load(); assert.equal(r.list().length,6);
  for(const l of r.list()) {assert.equal(r.locate(l.pose),l.id); assert.ok(l.adjacency.length>0);}
  for(const modify of [(d:any)=>d.locations[0].pose.x=NaN,(d:any)=>d.locations[0].researcher_location="hidden",(d:any)=>d.locations[0].adjacency.push("absent")]) {
    const d=r.snapshot();modify(d); assert.throws(()=>new WorldRegistry(d),/INVALID_WORLD_REGISTRY/);
  }
  assert.equal(r.locate({x:100,y:100}),"unknown");
  assert.equal(r.locate({x:1,y:1}),"unknown"); // Door transition, not a forced room assignment.
  assert.equal(r.locate({x:2,y:1,frame_id:"camera_link"}),"unknown");
  const copy=r.resolve("Room_A")!; copy.x=999; assert.equal(r.resolve("Room_A")!.x,2);
});
test("nearest threshold does not snap unknown space to a distant waypoint",()=>{
  const r=WorldRegistry.historicalWaypoints({P:{x:0,y:0}});
  assert.equal(r.locate({x:0.1,y:0}),"P"); assert.equal(r.locate({x:1,y:0}),"unknown");
});
test("runtime navigation facts require real successful status; timeout never proves blockage",()=>{
  const r=new RuntimeWorldState();
  r.observePose({x:0,y:0}); assert.equal(r.snapshot().last_observation_timestamp,null);
  r.finish(r.begin("X"),{status:"FAILED",reason:"TIMEOUT"});
  assert.deepEqual(r.snapshot().blocked_routes,[]); assert.deepEqual(r.snapshot().visited_locations,[]);
  r.finish(r.begin("Y"),{status:"SUCCEEDED",accepted:true,action_status:"SUCCEEDED"});
  r.finish(r.begin("Y"),{status:"SUCCEEDED",accepted:true,action_status:"SUCCEEDED"});
  assert.deepEqual(r.snapshot().visited_locations,["Y"]);
  r.finish(r.begin("Z"),{status:"FAILED",reason:"PATH_BLOCKED",blocked_route:"observed_route"});
  assert.equal(r.snapshot().blocked_routes.length,1);
});
test("navigation and listed positions share one registry; semantic labels remain empty",async()=>{
  const registry=load(), calls:any[]=[];
  // Explicit unit stub only, never used by middleware acceptance.
  const bridge={request:async(method:string,params:any)=>{
    calls.push({method,params});
    if(method==="inspect_room")throw new BridgeError("PERCEPTION_NOT_CONFIGURED");
    if(method==="navigate_to") return {status:"SUCCEEDED",accepted:true,action_status:"SUCCEEDED"};
    if(method==="get_robot_state") return {pose:{x:2,y:1},timestamp:{sec:1,nanosec:0}};
    return {robot_pose:{x:2,y:1},timestamp:{sec:1,nanosec:0},lidar:{available:false},camera:{available:false},semantic_objects:[]};
  }} as unknown as JsonlBridge;
  const g=new Ros2RobotGateway({registry,bridge}); await g.navigateTo("Room_A");
  assert.deepEqual(calls[0].params.pose,g.listKnownLocations().find(l=>l.id==="Room_A")!.pose);
  const w=await g.getWorldState(); assert.equal(w.current_location,"Room_A");
  assert.deepEqual(w.visited_locations,["Room_A"]); assert.deepEqual(w.semantic_objects,[]);
  assert.ok(!JSON.stringify(w).includes("researcher"));
  assert.equal((await g.inspectRoom("Room_A")).code,"PERCEPTION_NOT_CONFIGURED");
});
