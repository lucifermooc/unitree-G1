# unitree-G1：G1 导航后端 + 网页控制台 + 语义地图

**当前版本 v1.0.0**（2026-10-06）。接手前先读 `lightning_ws/CLAUDE.md`：开头是总目录，后面按编号展开问题记录、硬性规则、已知问题和排查方法。

## v1.0.0 状态

| 项目 | 结果（Thor 128.146 实测） |
|---|---|
| 到点精度（前台↔厕所各 5 次，按激光真实位置） | 平均 12.9 cm、最大 19.8 cm，航向 ≤2.8°，10/10 成功 |
| 导航方案 | SmacPlanner2D 全局规划 + Regulated Pure Pursuit 控制，起步和到点时原地转向（G1 实测能原地转） |
| 定位 | Lightning-LM（LIO + NDT）+ 一致性守护（冻结 / 回退） |
| 障碍物 | MID360 + D435 都走 STVL，动态障碍残影和近身噪点已清零 |

**已知问题**（详见 `lightning_ws/CLAUDE.md` 第 4 章）：
- 导航时 NDT 会沿走廊把定位拉偏（守护能冻结，但冻结后不会自动恢复，前端也不显示）。
- 定位重启总是从地图原点初始化：重启前机器人必须在原点、朝向约 0°，重启后要用激光核对。
- 到点后原地转向时身体会位移，是剩余到点误差的主要来源；RPP 不会局部绕障，有人挡路的情况还没测。
- 112.70 还是 09-30 的代码，开机自启 unit 指向 src，需要重新部署。

## 仓库结构

```
lightning_ws/                       G1 的 ROS2 工作区（Jazzy）
  G1_Communication_Documentation.md   前端 ↔ 机器人接口文档（已对照源码修订，含语义地图章节）
  CLAUDE.md                           工作手册（总目录 + 问题与解决记录 + 硬性规则 + 已知问题），先读这个
  AI_HANDOFF_*.md, launchreadmd.md    早期交接文档、手动启动命令
  tools/deploy/deploy_to_robot.sh     部署脚本：同步 → 停服务 → 只编译指定包 → 启服务 → 打印状态
  tools/field_test, tools/nav2_debug  现场测试和 costmap 排查工具
  src/
    lightning-lm, aid_navigation2, robot_bringup, aid_robot_py, aid_robot_msgs,
    g1_nav_bridge, livox_*, realsense-ros, librealsense, unitree_*, point_filter ...
                                    ← 原有后端（v1.0.0 对导航、定位、桥接、开机自启做了修改，见 CLAUDE.md）
    semantic_map_ros/               【新增】语义地图节点：一句话 → 位置点 → /nav_to_pose
    g1_web/                         【新增】网页控制台（www/ 静态文件，在电脑上打开即可，不用放到 Thor）
    g1_sim/                         【新增】无硬件仿真：模拟机器人 + 仿真 launch（只用于测试）
    test/                           【测试功能，独立成包】nav_goal_sim（到点精度仿真）、g1_vln、lightnav_server
tests/
  run_sim_e2e.sh                    一键全流程测试（编译 → 单元测试 → 仿真 → 浏览器端到端测试）
  e2e/console.e2e.js                浏览器端到端测试（Playwright）
```

## 在真机（Thor）上用

Thor 上开机自启两个 systemd 服务：`g1-robot`（整套栈，含 rosbridge 和建图预览）和 `g1-semantic-map`（语义地图）。
**部署后 Thor 上的 src 会被删除，运行时只引用 `install/`。**

```bash
# 在电脑上部署（同步 → 停服务 → 只编译指定包 → 启服务），部署真机前要先征得同意
bash lightning_ws/tools/deploy/deploy_to_robot.sh <Thor的IP> aid_navigation2 g1_nav_bridge

# 在 Thor 上管理服务（需要的话先 source install/setup.bash）
S=/opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script
bash $S/g1_service.sh restart     # 重启整套栈；status / stop / start / log 同理；第一次要 install
```

**重启会让定位从地图原点重新初始化**：重启前先把机器人放在原点、朝向约 0°，重启后核对定位是否正确。
语义地图的依赖和 Qdrant 见 `src/semantic_map_ros/README.md`。

网页控制台（`g1_web`）只是静态网页，**不用放到 Thor**，在自己电脑上打开：

```bash
python3 lightning_ws/src/g1_web/serve.py 8080
```

浏览器打开 `http://localhost:8080/?host=<Thor的IP>`（或打开后在连接地址里填 Thor 的 IP）。

网页控制台只保留接口文档里的功能：地图管理（使用/重命名/删除）、建图（实时预览、保存）、点位（在地图上拖动新建、编辑、删除）、
单点导航（暂停/继续/取消）、巡逻、禁行线、橡皮擦、模式切换、重定位（定位端已订阅 `/initialpose`，**未实机验证**），外加语义地图（搜索、一句话导航）。

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

已在 Thor（128.146）实机验证：建图、定位、Nav2 导航到点（10-06 测了 4 轮 40 次，过程和数据见 `lightning_ws/CLAUDE.md` 第 3.4 节）、
开机自启、语义地图服务（BGE-M3 从本地目录加载，Qdrant 共用 Thor 上已有的容器）。

仍未验证：`/initialpose` 重定位、导航中有人或障碍物挡路时 RPP 的表现、新导航方案的仿真（`src/test/nav_goal_sim` 没有按 v1.0.0 重跑）。

接口文档核对出的问题（旧文档与代码不一致处）列在接口文档末尾的《附录 A 修订记录》。
