# 交接说明（给接手的 Claude 会话）

先读：`README.md`（仓库结构、怎么跑、v1.0.0 状态）、`lightning_ws/CLAUDE.md`（总目录 + 用户的工作规则、问题记录、已知问题；**v1.0.0 以它为准**）、
`lightning_ws/G1_Communication_Documentation.md`（接口文档，已按源码修订，第五章是语义地图）。

## 用户和约定

- 用户是初学者：用中文、通俗、一步一步解释。
- 语义地图阶段约定过不修改原有后端文件；v1.0.0 为了到点精度、定位守护、开机自启已经修改了导航、定位、桥接和启动脚本（见 CLAUDE.md 第 3 章）。
  测试类功能仍然新增独立包，放在 `lightning_ws/src/test/`，不改现有代码。
- 遵守 `lightning_ws/CLAUDE.md`：不在本地编译，编译和测试 rsync 到 Thor 上做；rsync 要 `--exclude='bin/'`；
  部署真机前要用户同意；验证时不发导航目标、不发速度；重启用 `g1_service.sh restart`（或部署脚本 `tools/deploy/deploy_to_robot.sh`），
  **重启前机器人要在地图原点、朝向约 0°**（定位总从原点初始化），重启后用激光核对定位。
- 分支：`claude/inspiring-ptolemy-fm465d`。用户原始代码在 `g1-source` 分支（本分支的 `lightning_ws/` 与它逐字节一致，只多了三个新包和修订后的接口文档）。

## 已完成（云端仿真验证）

- `semantic_map_ros`：语义地图节点，服务 `/semantic_map/{search,go,rebuild}`；点位用数据库 `waypoint_node`，
  点位 JSON 可带 `description`；BGE-M3 + Qdrant；点位变化自动重建；`evaluate.py` 调阈值（默认 0.52）。
  其他模块（ASR）接入：调服务（`data` 可带 `{"text","source"}`）或发话题 `/semantic_map/text_in`；
  调试记录：`/semantic_map/debug`、`/semantic_map/history`、`~/maps/semantic_map_log.jsonl`、网页"调试记录"面板。
  ASR 那边的接口还没定，定了之后按对方格式适配（接口文档 5.5 / 5.6）。
- `g1_web`：网页控制台 `www/`（建图预览用原有的 `map_transform_node` 发布的 `/map_base64`）。
- `g1_sim`：模拟机器人 + `sim.launch.py`（真实 rosbridge / map_manager_server / waypoint_manage）。
- `bash tests/run_sim_e2e.sh`：36/36 通过；单元测试 5 项通过。

## 下一步（按顺序，都需要 Thor）

1. 只需把 `semantic_map_ros` rsync 到 Thor 的 `/opt/G1/lighting_ws/src/` 并只编译它。`g1_web` 只有静态网页，
   **不用放到 Thor**：在任意电脑上 `python3 lightning_ws/src/g1_web/serve.py 8080`，
   浏览器打开 `http://localhost:8080/?host=<Thor IP>`。建图预览 `/map_base64` 由原有的 `map_transform_node` 提供。
2. Thor 上装语义地图依赖（Jetson 版 torch、`requirements.txt`），`docker compose` 起 Qdrant，
   BGE-M3 模型可从电脑拷 `~/.cache/huggingface/hub/models--BAAI--bge-m3`。
3. Thor 上启动 `ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true`；网页按第 1 步在电脑上打开。
4. 真机验证（先征得用户同意）：建图时网页预览是否出图；地图/点位/禁行线/橡皮擦；
   语义搜索（只搜索不导航）；导航类功能按 CLAUDE.md 要求由用户在场时再测。
5. 用真实点位 + 真实说法跑 `python3 -m semantic_map_ros.evaluate --queries ...`，调整 `score_threshold`。

## 已知限制

- 重定位：Lightning 已订阅 `/initialpose`，**未实机验证**；`/start_init_pose` 仍无人订阅。
- `/patrol_control` 只有 `cmd`，巡逻圈数/时长未实现。
- 橡皮擦改的是地图文件，需重新 `mode_set localization` 才生效；这会重启定位并从地图原点初始化，机器人不在原点时定位会偏（CLAUDE.md 3.2.7）。
