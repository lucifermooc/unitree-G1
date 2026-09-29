# unitree-G1：G1 导航后端 + 网页控制台 + 语义地图

## 仓库结构

```
lightning_ws/                       G1 的 ROS2 工作区（Jazzy）
  G1_Communication_Documentation.md   前端 ↔ 机器人接口文档（已对照源码修订，含语义地图章节）
  CLAUDE.md, AI_HANDOFF_*.md, ...     原有的开发记录
  src/
    lightning-lm, aid_navigation2, robot_bringup, aid_robot_py, aid_robot_msgs,
    g1_nav_bridge, livox_*, realsense-ros, librealsense, unitree_*, point_filter ...
                                    ← 原有后端，原样保留，未做任何修改
    semantic_map_ros/               【新增】语义地图节点：一句话 → 位置点 → /nav_to_pose
    g1_web/                         【新增】网页控制台（www/ 静态文件 + 网页服务 launch）
    g1_sim/                         【新增】无硬件仿真：模拟机器人 + 仿真 launch（只用于测试）
tests/
  run_sim_e2e.sh                    一键全流程测试（编译 → 单元测试 → 仿真 → 浏览器端到端测试）
  e2e/console.e2e.js                浏览器端到端测试（Playwright）
```

## 在真机（Thor）上用

原有启动流程不变（`stop_all.launch.py` → `robot.launch.py`，rosbridge 默认已启动）。新增的两部分单独启动：

```bash
colcon build --packages-select semantic_map_ros g1_web      # 只编译新增的包
# 语义地图（先按 src/semantic_map_ros/README.md 装依赖、启动 Qdrant）
ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true
# 网页控制台
ros2 launch g1_web web.launch.py
```

浏览器打开 `http://<Thor的IP>:8080`，连接地址填 Thor 的 IP。

网页控制台只保留接口文档里的功能：地图管理（使用/重命名/删除）、建图（实时预览、保存）、点位（在地图上拖动新建、编辑、删除）、
单点导航（暂停/继续/取消）、巡逻、禁行线、橡皮擦、模式切换、重定位（机器人端暂未实现），外加语义地图（搜索、一句话导航）。

## 在没有机器人的电脑上跑仿真

```bash
bash tests/run_sim_e2e.sh                  # 全自动：编译、单元测试、启动仿真、浏览器测试
# 或者手动玩：
ros2 launch g1_sim sim.launch.py           # 然后浏览器打开 http://localhost:8080，连接 localhost
```

仿真里**真实运行**的：rosbridge（参数与真机相同）、`map_manager_server`（地图/点位/禁行线数据库）、
`waypoint_manage`（导航/巡逻调度、任务状态）、`map_transform`（建图预览 `/map → /map_base64`）、`semantic_map_server`、网页。
**模拟**的（`g1_sim/mock_robot`）：模式切换和保存地图（robot_status_manager）、Lightning 建图/定位、电池、Nav2 执行
（按 A* 路径走过去）、橡皮擦和禁行线地图。仿真地图是 `g1_sim/maps/office`（一层办公室）。

依赖：ROS2 Jazzy + rosbridge_suite + nav2_msgs、Docker（Qdrant）、Node.js + `npm i -g playwright`、
语义地图的 Python 依赖。仿真数据放在 `WORK_DIR`（默认 `/tmp/g1_sim_test`），不会碰 `~/maps`。

## 已验证 / 未验证

已在云端仿真验证（`tests/run_sim_e2e.sh`）：网页上的每个功能都经真实后端节点走通，结果通过后端服务回查；
语义地图用真实 BGE-M3 模型 + Qdrant。单元测试覆盖语义地图核心逻辑。

未验证（需要在 Thor 上做）：Lightning 真实建图下的建图预览、真实 Nav2 导航、
Thor 上 BGE-M3 的 GPU 推理和速度、国内网络下载模型。

接口文档核对出的问题（旧文档与代码不一致处）列在接口文档末尾的《附录 A 修订记录》。
