/** Image-only perception via official Agnes SDK. Never reads credentials or world files. */
import { readFile } from "node:fs/promises";
import { createInterface } from "node:readline";
import { createClient,memoryJournal } from "../../../third_party/agnes-harness/packages/sdk/src/index.node.ts";
const root="/data-ssd/zhongchengkai/agnes";
let client:any;
let stage="CONNECT";
function textOf(d:any):string {
  return typeof d==="string"?d:d?.text??(Array.isArray(d?.content)?d.content.map(textOf).join(""):Array.isArray(d)?d.map(textOf).join(""):"");
}
function jsonObjects(text:string):any[] {
  const results:any[]=[];let start=-1,depth=0,quoted=false,escaped=false;
  for(let i=0;i<text.length;i++){
    const c=text[i];
    if(quoted){if(escaped)escaped=false;else if(c==="\\")escaped=true;else if(c==='"')quoted=false;continue;}
    if(c==='"'&&depth){quoted=true;continue;}
    if(c==="{"){if(!depth)start=i;depth++;}
    if(c==="}"&&depth&&!--depth){try{results.push(JSON.parse(text.slice(start,i+1)));}catch{}}
  }
  return results;
}
async function connect(){
  if(client)return client;
  const owner=JSON.parse(await readFile(`${root}/.agh-home-step7/data/daemon/owner.json`,"utf8"));
  client=createClient({transport:{kind:"unix",path:owner.socketPath},auth:{kind:"local"},journal:memoryJournal()});
  await client.initialize();
  const config=await client.config.get();
  if(!config.configured || config.provider?.id!=="agnes-ai" || config.provider?.model!=="agnes-3.0-flash")throw Error("VLM_PROVIDER_NOT_ACTIVE");
  return client;
}
async function detect(request:any){
  const c=await connect();
  if(request.method==="audit")return {provider:"agnes-ai",model:"agnes-3.0-flash",configured:true,image_catalog_support:true};
  stage="SESSION_NEW";
  const session=await c.session.new({cwd:root,sessionKey:`phase10-image-${Date.now()}`});
  await session.attach();
  const remove=session.onPermissionRequest(async(r:any)=>({optionId:r.options.find((o:any)=>o.kind==="reject_once").optionId}));
  const iterator=session.events()[Symbol.asyncIterator]();
  let text="";
  let toolCalls=0;
  const reading=(async()=>{for(;;){const item=await iterator.next();if(item.done)break;
    if(item.value.type==="tool/call") {toolCalls++;void session.cancel().catch(()=>undefined);}
    if(item.value.type==="assistant/message"){
      const d=item.value.data;
      text+=textOf(d);
    }
  }})();
  try{
    stage="IMAGE_PROMPT";
    await session.prompt([{type:"text",text:"Image-only visual inspection. Do not call tools, do not use any task/world/session knowledge. Identify ONLY visibly supported demo objects: person, fire_extinguisher, toolbox, hazard_marker. Empty list if absent. Return ONLY JSON {\"detections\":[{\"label\":\"person\",\"confidence\":0.0,\"bbox\":[x1,y1,x2,y2]}]}. Bbox coordinates normalized 0..1. Confidence must reflect visual uncertainty; do not force high confidence. A red object alone is NOT proof of an extinguisher. Person means visible human-shaped figure, not confirmed researcher identity."},
      {type:"image",data:request.png,mimeType:"image/png"}],{signal:AbortSignal.timeout(45000)});
    stage="EVENT_READING";
    await iterator.return?.();await reading;
    if(toolCalls){stage="TOOL_CALL_FORBIDDEN";throw Error("VLM_TOOL_CALL_FORBIDDEN");}
    stage=text?"JSON_PARSE":"EMPTY_ASSISTANT_OUTPUT";
    const output=jsonObjects(text).findLast((o:any)=>Array.isArray(o.detections));
    if(!output){stage="NO_VALID_DETECTIONS_JSON";throw Error("VLM_INVALID_OUTPUT");}
    if(!Array.isArray(output.detections))throw Error("VLM_INVALID_OUTPUT");
    const detections=output.detections.filter((d:any)=>["person","fire_extinguisher","toolbox","hazard_marker"].includes(d.label)
      &&Number.isFinite(d.confidence)&&d.confidence>=0&&d.confidence<=1&&Array.isArray(d.bbox)&&d.bbox.length===4
      &&d.bbox.every((n:any)=>Number.isFinite(n)&&n>=0&&n<=1)&&d.bbox[0]<d.bbox[2]&&d.bbox[1]<d.bbox[3]);
    return {provider:"agnes-ai",model:"agnes-3.0-flash",detections,tool_calls:toolCalls};
  }finally{remove();await iterator.return?.();await reading;await session.detach?.();}
}
try{
  for await(const line of createInterface({input:process.stdin})){
    const request=JSON.parse(line);
    try{const result=await detect(request);console.log(JSON.stringify({id:request.id,ok:true,result}));}
    catch{console.log(JSON.stringify({id:request.id,ok:false,error:{code:"VLM_UNAVAILABLE_OR_INVALID_OUTPUT",stage}}));}
  }
}finally{await client?.close();}
