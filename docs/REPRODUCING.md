# Agnes Embodied Rescue 复现指南

本指南区分两条路径：先用干净源码运行离线回归，再在准备好模型、机器人环境和本地运行配置后启动实时仿真。两条路径的成功条件不同。

| 路径 | 需要什么 | 完成后能确认什么 |
| --- | --- | --- |
| 离线最小验证 | Node.js 24.10+；Python 3.12 为已测版本 | 核心纯逻辑可从源码运行，不发起模型请求或机器人运动 |
| 实时仿真演示 | Linux、模型授权、ROS2 / Nav2 / Gazebo、已配置的本机运行环境 | 原生聊天与真实仿真工具连通，地图和 Camera 提供当前数据 |

现有演示机的实时链路已验证，全新机器完整部署尚未独立验证。当前启动、监控、视觉和地图导入代码仍有固定工作区路径，办公室 AMCL overlay 也不在公开源码内。以下步骤将这些依赖显式列出，不将源码克隆等同于完成部署。

## 1 获取源码并进行离线验证

```bash
git clone https://github.com/sevenboom77/Agnes_Embodied_Rescue.git
cd Agnes_Embodied_Rescue

node --version
python3 --version

node --test \
  services/robot-mcp/tests/navigation-lifecycle.test.ts \
  services/robot-mcp/tests/perception-output.test.ts \
  services/robot-mcp/tests/vlm-turn.test.ts \
  packages/embodied-rescue-plugin/tests/runtime-entry.test.ts

python3 -B -m unittest discover \
  -s services/robot-mcp/tests -p 'test_doorway_navigation.py'
```

已在不含 `node_modules`、`profiles`、`artifacts` 和已安装 ROS 的源码快照中执行：19 项 Node 回归与 6 项 Python 回归通过。测试使用明确的单元测试替身，不读取现场相机、驱动机器人或调用模型。

完整 Harness 源码直接放在 `third_party/agnes-harness`，不需要 `git submodule update`。源码不包含工具链安装包、账号数据或运行历史。

不要把根目录的 `pnpm test` 当作这一步的等价替代：它会选择更多测试文件，其中部分是依赖 ROS、Gazebo 或阶段性运行环境的集成测试。需要真实模型或机器人栈的检查应单独安排。

## 2 了解实时环境和路径要求

| 组件 | 演示环境与源码依据 |
| --- | --- |
| 操作系统 | Ubuntu / Linux x86_64；Pixi 定义为 `linux-64` |
| Node.js | 24.10.0；[根清单](../package.json)要求至少 24.10 |
| pnpm | 10.34.5 |
| Pixi | 0.81.0 |
| 机器人环境 | ROS2 Jazzy、Nav2、Gazebo Sim 8；见 [pixi.toml](../runtime/ros2-jazzy/pixi.toml) 与 [pixi.lock](../runtime/ros2-jazzy/pixi.lock) |
| 模型 | 自有授权的 Agnes Provider，任务与视觉使用 `agnes-3.0-flash` |
| ROS 隔离 | `ROS_DOMAIN_ID=106` |
| Gazebo 隔离 | `GZ_PARTITION=agnes_final_rescue` |

离线最小验证可以在普通源码目录运行。**当前完整演示代码预期工作区为 `/data-ssd/zhongchengkai/agnes`。**如果你使用其他目录，需要先迁移代码中的绝对路径及配置，不能仅设置一个环境变量就认为完成适配。不要为匹配路径覆盖已有项目。

可用以下只读命令定位路径绑定，按文件逐项核对：

```bash
git grep -n '/data-ssd/zhongchengkai/agnes' -- \
  scripts services demo packages simulation runtime
```

Windows 可用浏览器访问部署在 Linux 的页面。Windows 本机直接部署完整机器人栈不在当前验证范围内。

## 3 准备工具链和锁定依赖

工具链不随仓库分发。按演示版本从官方发行渠道准备 [Node.js](https://nodejs.org/dist/v24.10.0/) 和 [Pixi](https://github.com/prefix-dev/pixi/releases/tag/v0.81.0)，校验官方提供的校验信息，并按下列结构放在项目内：

```text
tools/
├── node/bin/node
├── node/bin/pnpm
└── pixi/bin/pixi
```

Node 发布包不一定直接包含可用的 pnpm 命令；请通过对应 Corepack / pnpm 工具准备 `pnpm@10.34.5`。项目脚本会使用这些路径，不会因系统 PATH 上另有 Node 或 Pixi 就自动替代所有入口。

工具就绪后，从项目根目录运行：

```bash
source scripts/use-local-toolchain.sh
node --version
pnpm --version
tools/pixi/bin/pixi --version

pnpm --dir third_party/agnes-harness install --frozen-lockfile
pnpm --dir services/robot-mcp install --frozen-lockfile

tools/pixi/bin/pixi install --locked \
  --manifest-path runtime/ros2-jazzy/pixi.toml

pnpm --dir third_party/agnes-harness --filter @agnes/cli build:local
```

`use-local-toolchain.sh` 将 pnpm、npm 和临时文件的相关目录限制在项目中，不修改 shell 启动文件。原生 Harness 的额外构建要求见其[安装指南](../third_party/agnes-harness/docs/guide/install.zh-CN.md)。

环境交互 MCP 复用 `services/robot-mcp/node_modules` 的 MCP SDK 与 Zod，所以 Robot MCP 的依赖安装不能省略。

### 办公室的额外依赖

- **AMCL overlay：**当前栈在办公室定位模式检查 `runtime/ros2-jazzy/overlays/amcl/lib/nav2_amcl/amcl`。这个 overlay 是本机构建产物，不包含在源码或当前 Pixi 清单中，需准备与 Jazzy 环境匹配的 AMCL 构建；检查位置见 [stack.py](../scripts/phase9d/stack.py)。
- **体积火焰插件：**办公室视觉使用项目 C++ / GLSL 插件。构建入口为 `python3 -B simulation/visual_assets/volumetric_fire/build.py`；需要对应 Gazebo / Ogre 开发依赖与着色器校验工具，详见[插件说明](../simulation/visual_assets/volumetric_fire/README.md)。生成的 `build/` 不随源码上传。
- **GPU 渲染与相机依赖：**在目标主机上核查 Gazebo / Ogre 渲染、相机订阅、图像编码依赖及模型网络连接；仅成功安装 Pixi 不能替代这些检查。

这些是新机器恢复环境时的独立步骤，不应通过复制别人的私有 Profile 或整个运行目录解决。

## 4 配置真实 Agnes 与 MCP

### 模型与 Profile

项目使用 `AGH_HOME=<项目根目录>/.agh-home-step7`、Profile `local-dev`。先按 Harness 的[首次运行说明](../third_party/agnes-harness/docs/guide/quickstart.zh-CN.md)完成原生配置流程，再配置你自己的 Agnes 账号与模型权限。

当前视觉适配器要求 Provider ID 为 `agnes-ai`、模型为 `agnes-3.0-flash`；仅配置一个可对话的通用模型不能替代这个检查。API Key 通过原生配置机制安全保存，不写入 README、脚本或 Git。

Rescue Plugin 来源为 `file:./packages/embodied-rescue-plugin`，需按 Harness 流程检查、安装、信任并启用。它提供认知工具和 `sensor-image` 视觉预设，不直接替代 Robot MCP 的运动接口。

### 两个真实 MCP 服务

下表对应 [实时资源准入代码](../scripts/interactive/ensure-real-runtime.mjs)中的 SDK 服务定义。通过 Harness 支持的 MCP 配置流程设置，不要直接覆盖未知格式的私有配置文件。

| 字段 | Robot MCP | 环境交互 MCP |
| --- | --- | --- |
| serverId | `robot-mcp` | `envdoor` |
| transport | `stdio` | `stdio` |
| executable | `<根目录>/tools/node/bin/node` | `<根目录>/tools/node/bin/node` |
| args 的唯一入口 | `<根目录>/scripts/final-rescue/mcp-launch.mjs` | `<根目录>/scripts/final-rescue/environment-interaction-launch.mjs` |
| secretBinding | `{ "kind": "none" }` | `{ "kind": "none" }` |

请把 `<根目录>` 替换成已核对的项目绝对路径，保持启动器继承本项目隔离环境。有关原生配置接口见 [Harness MCP 指南](../third_party/agnes-harness/docs/guide/mcp.zh-CN.md)。

核查条件是 Plugin 已信任且运行、两个 MCP 服务均 ready，以及 Robot MCP 暴露八个预期工具。**仅有八个同名工具不足以证明它是真实 ROS 后端**，还应核对上述启动器与 `ROBOT_GATEWAY=ros2`。

历史 `scripts/step7-prepare.mts` 会注册 Mock Robot MCP；不要把它用于本节的真实环境初始化。`configure-step7-agnes-provider.sh` 也会停止和重启项目旧服务，不应在已有活动会话或演示时直接运行。

## 5 启动已完成配置的实时环境

本节会启动进程或调整本项目运行状态，不属于离线测试。先确认没有活动任务、导航 Goal 或被持有的运动 lease；已有服务应复用，不根据端口占用就停止未知 PID。

### 原生 Web

在项目根目录的一个终端启动：

```bash
bash scripts/start-step7-agnes.sh
```

它使用项目专属的运行目录，并提供原生 Harness Web 及 `4177` 入口。若已由项目管理器运行，不要重复执行。

### 仿真与真实资源准入

在另一个终端，从已完成配置、准备冷启动的环境使用：

```bash
bash scripts/interactive/start.sh
```

这个入口并非“仅启动页面”：它会启动或复用仿真栈、配置或启用项目 Plugin / MCP、重新连接受管服务、执行真实 Provider 与工具探测、启动 Camera adapter，并更新本地交互状态。探测需要模型授权和网络连接。

现有准入探测包含历史门 ID `D_A_Link`，与当前地图不匹配时应诊断探测结果，不能据此认定新地图的所有门不可用，也不能盲目循环重跑。

单独的 `bash scripts/final-rescue/start-stack.sh` 只启动机器人栈，不等于 Provider、Plugin、相机展示与原生 Session 都准备完成。

### 演示页面与只读监控

```bash
python3 scripts/demo-ui-final/start.py
```

该 UI-only 入口启动或复用页面、会话观察器和 ROS 监控，不初始化模型凭据，不重置仿真，也不发导航命令。它要求当前工作目录为上述固定项目根目录。不要为启动页面额外附加 `--restart-ui`。

访问：**http://127.0.0.1:4177/demo**

当前 Session 由原生会话观察器与页面关联，工具结果读取自真实 Session。不同入口都显示 ready，不代表当前 Mission 成功。

## 6 远程浏览器与 WebSocket

将下列占位内容替换成你自己的 Linux 用户和服务器地址：

```bash
ssh -N -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:4177:127.0.0.1:4177 YOUR_USER@YOUR_SERVER
```

随后在浏览器访问 `http://127.0.0.1:4177/demo`，并保持 SSH 连接。

**当前源码已通过 `/agh-rpc` 代理原生 WebSocket，正常仅需转发 4177。**见 [web.mjs](../scripts/agnes-proxy/web.mjs) 和 [websocket-proxy.mjs](../scripts/agnes-proxy/websocket-proxy.mjs)。4188 是旧 Dashboard / API 入口，只有使用对应接口时才需额外转发。

若使用旧部署且浏览器实际连接的是 daemon 的独立动态端口，再按实际端口追加转发。不要把旧报告中的 daemon 端口固定写入配置，也不要为远程访问开放未鉴权的公网 ROS 或模型服务。

## 7 逐层核查后发起任务

先检查只读状态接口：

```bash
curl --fail --silent --show-error \
  http://127.0.0.1:4177/api/demo-ui/state
```

重点核对：

1. 原生会话观察器已连接，Plugin 和 MCP 状态正常。
2. ROS 状态和 map / TF 位姿新鲜；机器人在当前地图中的位置合理。
3. Camera 标记为 live，并持续收到新帧，而非只显示上一张图片。
4. 导航为 IDLE、无活动 Goal、运动 lease 空闲。

这份状态响应包含本机 Session 等运行信息，不应原样作为公开问题附件。

在原生聊天先发送：

> 现在机器人在哪里？

应看到真实 `get_robot_state` 工具结果，并与地图位置对应。然后，在确认环境安全后发送：

> 请找到一个待救援人员。

开门、入室、接近、多视角复核或停止如属于你的验收目标，应在任务中明确提出，并保留追加指令。成功条件应来自目标相关的工具终态、当前位置、新相机帧与停止状态，而非只看模型结束了一轮对话。

## 8 上传骨架和切换地图

- 从 [office_skeleton.json](../maps/imported/office_fire/office_skeleton.json) 或历史 [lotus_skeleton.json](../maps/imported/lotus_demo/lotus_skeleton.json)了解数据格式。
- 在页面选择本地 JSON，命名、上传并查看静态预览。骨架只描述空间先验，不包含待搜寻人员或运行中门状态的答案。
- 保存和预览不会激活物理场景；实际切换需选择地图并显式确认，通过空闲、Goal、运动锁和会话检查。
- 切换后核查当前地图、出生点、TF 与 Camera。刷新页面不应自动重试一次中断的物理切换。

在当前固定工作区内可单独运行只读骨架校验：

```bash
python3 -B scripts/map-import/map_import.py validate \
  maps/imported/office_fire/office_skeleton.json
```

`validate` 不启动 Gazebo，也不写入激活指针。该导入器仍限制输入和输出必须位于固定工作区内；它不属于第 1 节已验证可在任意目录执行的离线测试。

## 9 常见问题

| 现象 | 优先核查 |
| --- | --- |
| 页面可开但 Agnes 不能对话 | Provider 是否配置、原生 Session 是否连通、`/agh-rpc` 是否完成 WebSocket 握手；旧部署再检查动态端口 |
| Chat 可用但没有机器人能力 | 是否使用本项目 `local-dev` Profile、Rescue Plugin 是否已启用、两个 MCP 是否为真实启动器 |
| 第一视角黑或不刷新 | 先核对帧时间与 `/api/demo-ui/camera.jpg`；HTTP 图片可用时优先排查呈现链路，不直接认定传感器损坏 |
| 办公室定位无法启动 | 检查 AMCL overlay、当前场景参数与 map / odom / base_link 变换 |
| 场景切换被拒绝 | 检查活动 Goal、运动 lease、原生待授权项及观察器时效，不跳过检查 |
| `/demo-project-film-*` 或提交材料页面缺失 | 对应媒体或材料位于未分发的 `artifacts`；使用[公开视频](https://www.bilibili.com/video/BV1F1J66YEkB/) |
| 导航超时或返回部分完成 | 查看实际 Nav2 终态与操作所有权；不要同时提交第二个 Goal，也不要把超时自动解释为到达或碰撞 |

## 10 记录结果与提交问题

先注明你验证的是离线逻辑、资源准备还是实时任务，并记录代码提交、依赖版本、当前地图及可重现步骤。实时任务应区分组件 READY、模型回合结束、工具终态和 Mission 结果。

提交问题前移除 API Key、GitHub Token、原生认证信息、完整会话数据库、个人信息及未审查的 trace。停止或恢复操作仅针对可确认归属的本项目进程；保持 ROS domain 与 Gazebo partition 隔离。

[返回项目首页](../README.md) · [上游来源与适配说明](../UPSTREAM.md)
