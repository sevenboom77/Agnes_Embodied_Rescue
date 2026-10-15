# Agnes Embodied Rescue

基于 Agnes Harness 原生 Agent、Rescue Plugin、Robot MCP、ROS2/Nav2 与 Gazebo 的具身救援演示工程。

## 当前交付

- 官方原生 Agnes Web Chat，与真实 Provider、演示 Session 和 Robot MCP 的 8 个公开工具关联；没有仿造聊天界面。
- 相机优先的响应式监控：宽屏为原生聊天、放大的 4:3 第一视角与地图三列，窄屏优先展示相机。实时 ROS2 /map、TF 位姿、导航状态与 Gazebo Camera 同帧 JPEG 预览保留；无损 PNG 供视觉与核查，失联不把旧画面标为 LIVE。
- 地图骨架 JSON 上传、命名、保存、静态预览；必须显式确认并通过空闲/运动锁检查才切换物理场景。
- 经典地图、非官方简化莲华古城骨架；导入场景使用项目自制石材 PBR 纹理、米制 UV 与自然光照，地图采用墨黑实心粗线。
- 导入地图人物使用项目自制成人模型，包含四肢、衣着和面部细节；Camera 640 × 480 / 30 Hz。
- 新增室内办公室火灾地图：按用户草图搭建四个房间，机器人从底部面向室内出生；左侧三排、共六套电脑工位，封闭吊顶、灰色地毯、白墙及玻璃门参考用户提供的办公室视频。平面尺寸推定为 18 × 16 米，不是实测复刻。
- 办公室四扇门初始关闭，使用可开启的真实侧铰链；家具碰撞同步进入导航栅格。墙面使用多形态上升火舌，吊顶使用横向翻卷火带、柔和烟雾与暖色火光；移除了黑洞状焦黑贴片。莲华火灾地图仍可选择。程序资产来源及边界见 simulation/visual_assets/office_README.md 与 fire_README.md；均为视觉演示，不计算燃烧、温度或烟气传播。
- 原有 seed=1185 入口、随机人员/物品布置和救援 API 保留。

这是当前服务器的完整源码快照，不是包含账户凭据和已安装环境的机器镜像。完整 Harness 源码已直接纳入 third_party/agnes-harness，不依赖空的 Git submodule 目录。

## 代码结构

| 路径 | 内容 |
| --- | --- |
| packages/embodied-rescue-plugin | 救援 Plugin、技能与测试 |
| services/robot-mcp | 固定 8 工具、ROS 网关、视觉与安全控制 |
| services/environment-interaction-mcp | 场景交互 MCP |
| scripts/demo-ui-final | 原生会话、ROS 监控、安全场景切换与 UI 测试 |
| demo/final-dashboard | 双栏实时演示页面及 API |
| scripts/map-import | 骨架校验与 Gazebo/地图编译 |
| scripts/interactive | 随机场景、真实 Camera、任务执行与验收 |
| simulation | 经典世界、导航配置与场景模板 |
| maps/imported | 简化莲华等静态骨架与地图资产 |
| runtime/ros2-jazzy | Pixi 环境定义与锁文件 |
| third_party/agnes-harness | 固定上游基线及当前项目集成补丁，来源见 UPSTREAM.md |

上游来源和许可证见 UPSTREAM.md 以及上游目录中的 LICENSE。项目自有代码不因本次上传自动新增许可证授权。

## 环境与依赖

原演示环境为 Ubuntu linux-64，项目路径 /data-ssd/zhongchengkai/agnes；不少运行脚本仍显式使用此路径。移动到其他路径前需要审计配置，不能直接宣称跨路径一键运行。

当前使用 Node.js 24.10.0、pnpm 10.34.5、Pixi 0.81.0。安装后的 tools/node、tools/pixi、node_modules、Pixi 环境、额外 AMCL overlay 和缓存不进入仓库，应由使用者按对应官方发行包和锁文件重新安装。Harness 的构建说明见其 README；根 package.json 是本工程清单。

依赖安装可在已准备好对应工具链后进行：

~~~bash
cd /data-ssd/zhongchengkai/agnes
source scripts/use-local-toolchain.sh
pnpm --dir third_party/agnes-harness install --frozen-lockfile
pnpm --dir services/robot-mcp install --frozen-lockfile
tools/pixi/bin/pixi install --locked --manifest-path runtime/ros2-jazzy/pixi.toml
~~~

请依 Harness 自身构建说明生成原生 Web/CLI 构建产物。Provider/API Key、账户 Profile、daemon discovery、Session 和运行指针必须在本机安全配置/初始化；不能从仓库取得。旧阶段的 provider 配置脚本会停止本项目旧服务，不要在正在演示的机器上不经核对直接运行。

## 当前演示机入口

现有服务器已配置好的运行环境不需要因 Git 上传重启。

- 真实页面：http://127.0.0.1:4177/demo
- 原 Dashboard 4188 保留，主入口重定向到 /demo。
- UI-only 启动：python3 scripts/demo-ui-final/start.py（不初始化 Provider，不启动或重置机器人栈）。
- 原 seed=1185 完整任务入口：scripts/demo-rescue/start.sh；会管理项目运行态，使用前读脚本并核实安全状态。
- 原栈启动：scripts/final-rescue/start-stack.sh；不要对已运行的栈重复启动。
- 相机、Agent 与仿真由各自现有入口管理；UI READY 不代表 Mission SUCCEEDED。

Windows PowerShell 的页面转发：

~~~powershell
ssh -N -L 127.0.0.1:4177:127.0.0.1:4177 -L 127.0.0.1:4188:127.0.0.1:4188 guoyu@192.168.97.11
~~~

原生 Chat 还需要当前 Agnes WebSocket 动态端口转发。以页面断连提示/本机 discovery 的实际端口为准，不硬编码历史端口。示例：ssh -N -L 127.0.0.1:实际端口:127.0.0.1:实际端口 guoyu@192.168.97.11。

## 验证与已知限制

安全的源码检查包括 Node syntax check、Plugin/MCP 单元测试及静态骨架编译。部分历史 UI/地图回归脚本依赖服务器 artifacts 内的旧校验报告；这些报告不是源代码，不随本次上传。不能把缺失历史证据的干净克隆当成已完成集成部署。

本轮视觉统一通过 14 项专项静态检查、25 项既有 UI 回归，并实测 Camera 恢复正常。同 seed 受控重载后 map SHA、物品布置、Session 均不变，Nav2 IDLE、zero velocity。没有因发布执行导航任务。

2026-10-12 室内办公室地图已通过静态编译、材质与家具检查，并通过正常场景切换流程启动真实 Gazebo/ROS。四扇门逐一实测打开至约 90°、有效通行宽度约 1.177 米，测试后均恢复关闭；机器人保持出生点。20 秒真实 Camera 采样约 27.0 FPS，页面观察约 26 FPS。Agnes 图像识别已加入 door 类别，并用当前真实 Camera 图片完成识别；识别仍只使用 Agnes 模型，禁止将火光颜色当作门。门验证为直接物理集成测试，未据此声称完整 Agnes 自主开门导航任务已通过。运行证据位于 artifacts/indoor-fire-20261012。

后续火焰与布局优化通过 22 项材质/几何/导航回归，并以相同 seed、人员物品位置、导航栅格受控重载。20 秒真实相机采样约 26.2 FPS。已在 390、1366、1920 像素宽度核对相机与容器均为 4:3、无左右留白或横向溢出；布局检查恢复了用户原始窗口尺寸。新验证证据位于 artifacts/office-fire-v2b-20261012。

已知：短自然语言任务有真实成功记录，但固定完整救援流程曾 PARTIAL（返航 ABORTED/104），不能宣称完整流程稳定通过。莲华是非官方简化骨架，不含游戏完整旋转门、可破坏墙和高差机制。运行态/历史证据保留在授权服务器项目目录，不把 trace、图像、密钥当作源码发布。

## 安全边界

仅操作项目拥有的进程；ROS_DOMAIN_ID=106，GZ_PARTITION=agnes_final_rescue。禁止 sudo、停止未知 PID、干扰其他 ROS domain。切换地图必须满足真实任务/Goal 空闲、运动 lease 可用，并验证当前地图/TF/Camera；网页刷新不得自动重复导航或激活。

.gitignore 排除了项目私有目录、GitHub 登录凭据、Provider 数据、缓存、日志、历史备份和机器安装包。上游保留其公开的测试用假 Token/TLS fixture；它们不是本项目实际凭据，不能用于生产。
