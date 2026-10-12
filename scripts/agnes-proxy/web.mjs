// Project-local launcher: official assets/auth/session unchanged; supported createServer BFF hook.
await import('/data-ssd/zhongchengkai/agnes/third_party/agnes-harness/node_modules/.pnpm/tsx@4.23.13/node_modules/tsx/dist/loader.mjs')
const {runWebCommand}=await import('/data-ssd/zhongchengkai/agnes/third_party/agnes-harness/packages/cli/launch/web-command.ts')
const {createWebServer}=await import('/data-ssd/zhongchengkai/agnes/third_party/agnes-harness/packages/web-server/src/server.ts')
const {handleDemoUi}=await import('../../demo/final-dashboard/demo-ui-server.mjs')
const {handleMapCatalog}=await import('../../demo/final-dashboard/map-catalog.mjs')
const root='/data-ssd/zhongchengkai/agnes',local=root+'/third_party/agnes-harness/packages/cli/dist/local'
const resources={mode:'package',root:local,daemonEntry:root+'/scripts/agnes-proxy/daemon.mjs',workerEntry:root+'/scripts/agnes-proxy/worker.mjs',webRoot:local+'/web'}
const createServer=options=>createWebServer({...options,handleAdmin:async(req,res)=>{
 const url=new URL(req.url||'/',options.origin)
 if(await handleMapCatalog(req,res,url,{origin:options.origin}))return true
 if(url.pathname==='/demo'){url.pathname='/';return handleDemoUi(req,res,url)}
 if(url.pathname.startsWith('/api/demo-ui/')||['/demo-ui.css','/demo-ui.js'].includes(url.pathname))return handleDemoUi(req,res,url)
 // Only the existing static-preview upload endpoint. Never proxy physical activation or mission commands.
 if(url.pathname==='/api/map/upload'&&req.method==='POST'){
  if(req.headers.origin!==options.origin||!['same-origin',undefined].includes(req.headers['sec-fetch-site'])){res.writeHead(403);res.end('same-origin required');return true}
  let size=0;const chunks=[]
  for await(const chunk of req){size+=chunk.length;if(size>3*1024*1024){res.writeHead(413);res.end('upload too large');return true}chunks.push(chunk)}
  try{const upstream=await fetch('http://127.0.0.1:4188/api/map/upload',{method:'POST',headers:{'content-type':'application/json'},body:Buffer.concat(chunks),signal:AbortSignal.timeout(65000)});res.writeHead(upstream.status,{'content-type':'application/json','cache-control':'no-store'});res.end(Buffer.from(await upstream.arrayBuffer()))}catch{res.writeHead(503);res.end('preview backend unavailable')}
  return true
 }
 return options.handleAdmin?options.handleAdmin(req,res):false
}})
await runWebCommand(process.argv.slice(2),{env:process.env,cwd:process.cwd(),resources,createServer,signals:process})
