# semantic_map_ros：语义地图

完整的数据流（每一步的数据格式）见 [DATA_FLOW.md](DATA_FLOW.md)。

说一句话（"我想喝水"），在当前地图的**位置点**里找到意思最接近的一个（"茶水间"），并让机器人过去。

- 点位就是地图数据库里的位置点（`waypoint_node`，前端"点位"页面管理），**不需要单独维护数据**。
- 给点位写上**描述**（`description` 字段），匹配会准很多。
- 只通过现有接口和后端交互，**不修改任何原有节点**：读 `/get_current_map_id`、`/get_map_point_list`，导航发 `/nav_to_pose`。

## 工作原理

```
【建库】（点位有变化时自动进行）
  /get_map_point_list → 每个点位：{名称, 描述, 坐标}
        │
        ├─ "名称。描述" ──> BGE-M3 模型 ──> 1024 个数字（向量）
        └─ 整个点位（含坐标）作为附带信息（payload）
                     ↓
  Qdrant 集合 semantic_map_<地图id>：每个点位一条 [id | 向量 | payload]

【查询】
  "我想喝水" ──> BGE-M3 ──> 向量 ──> Qdrant 找最相似的点位（余弦相似度）
             ──> 分数 ≥ 阈值？ ──是──> 从 payload 取坐标 ──> /nav_to_pose ──> waypoint_manage 调 Nav2
                               └─否──> 返回"没找到"，机器人不动
```

## 接口

详见 `lightning_ws/G1_Communication_Documentation.md` 第五章。

| 服务 | 类型 | 作用 |
|---|---|---|
| `/semantic_map/search` | `aid_robot_msgs/srv/SetString` | `data` = 一句话；`message` = JSON 结果（best、candidates、found） |
| `/semantic_map/go` | `aid_robot_msgs/srv/SetString` | 搜索 + 找到就发导航 |
| `/semantic_map/rebuild` | `std_srvs/srv/Trigger` | 手动重建当前地图的语义库 |
| `/semantic_map/history` | `std_srvs/srv/Trigger` | 最近 200 条调试记录（JSON 数组） |

| 话题 | 类型 | 作用 |
|---|---|---|
| `/semantic_map/text_in` | `std_msgs/msg/String` | 输入：ASR 等模块往这里发一句话（默认只搜索，`text_in_action:=go` 找到就导航） |
| `/semantic_map/debug` | `std_msgs/msg/String` | 输出：每次请求一条 JSON（来源、原文、原文问题、前 3 名得分、结果、耗时） |

`data` 可以带来源：`{"text": "我想喝水", "source": "asr"}`。调试记录同时写进 `~/maps/semantic_map_log.jsonl`，
网页"点位与语义"页的"调试记录"实时显示所有来源的请求。

## 部署（Thor / 电脑都一样）

1. **Python 依赖**（装到 ROS 用的 Python 里）：
   ```bash
   # 电脑（没有 NVIDIA 显卡）先装 CPU 版 torch：
   pip install --break-system-packages torch --index-url https://download.pytorch.org/whl/cpu
   # Thor 上装 NVIDIA 提供的 Jetson 版 torch（不要用上面这行）
   pip install --break-system-packages -r src/semantic_map_ros/requirements.txt
   ```
2. **启动 Qdrant**（向量数据库，数据放在 `~/maps/qdrant_storage`，重启不丢）：
   ```bash
   docker compose -f src/semantic_map_ros/docker/docker-compose.yml up -d
   ```
3. **编译**：`colcon build --packages-select semantic_map_ros`
4. **下载模型**（约 2.3 GB，只需一次；国内网络先 `export HF_ENDPOINT=https://hf-mirror.com`）：
   第一次启动节点时自动下载到 `~/.cache/huggingface/`。Thor 不方便联网时，把电脑上下好的
   `~/.cache/huggingface/hub/models--BAAI--bge-m3` 整个目录拷过去即可。
5. **启动**（和 `robot.launch.py` 分开启动）：
   ```bash
   ros2 launch semantic_map_ros semantic_map.launch.py                # 电脑 / CPU
   ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true # Thor / GPU
   ```
   看到 `embedding model loaded` 就可以用了。

### Thor 实际部署记录（2026-09-29，JetPack 7 / R38.2，CUDA 13）

- **torch**：官方源 download.pytorch.org 国内超时，用 NVIDIA 的源：
  `pip install --user --break-system-packages -c ~/semantic_map_constraints.txt torch --index-url https://pypi.jetson-ai-lab.io/sbsa/cu130/+simple/ --extra-index-url https://pypi.tuna.tsinghua.edu.cn/simple`
  （实装 torch 2.14.0+cu130，自带 CUDA 运行库，不用装 CUDA 工具包）。
- **约束文件** `~/semantic_map_constraints.txt`：`numpy==1.26.4`（系统 OpenCV 依赖 numpy 1.x，`map_transform_node` 要用）、
  `setuptools<80`（colcon 要求；torch 会顺带装 setuptools 84，必须挡住）、`torch==2.14.0`。
  其余依赖用清华源 `-i https://pypi.tuna.tsinghua.edu.cn/simple`。
- **Qdrant**：Thor 上已有一个容器 `qdrant`（6333，开机自启），直接共用，**不要**再 `docker compose up`（端口冲突）。
- **模型**：hf-mirror 的大文件会跳到 xethub，新版 huggingface_hub 走 Xet 协议会 401 / 卡住。用 curl 下载到缓存
  （脚本 `/opt/G1/bags/replay/fetch_bge.sh`），固定路径 `~/models/bge-m3` 链到 snapshot 目录。
  **启动时必须给本地目录**：给仓库名的话，离线模式会因 onnx/imgs 没下而报 IncompleteSnapshotError，联网时则会多下 2.3 GB onnx。
  ```bash
  HF_HUB_OFFLINE=1 ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true model_name:=$HOME/models/bge-m3
  ```
- 实测：加载 ~10 s，单句编码 ~18 ms，显存 ~1.2 GB。

## 调阈值

`config/semantic_map.yaml` 里的 `score_threshold`（默认 0.52）：最高分低于它就算"没找到"。
用你们的真实点位和真实会说的话测一下：

```bash
cp src/semantic_map_ros/config/eval_queries_example.json my_queries.json   # 改成你们的句子
python3 -m semantic_map_ros.evaluate --queries my_queries.json             # 读 ~/maps/db.sqlite 的当前地图
```

它会打印每句话的第一名和分数，以及推荐阈值。经验：
- 打招呼、闲聊（"你好"）不要交给语义地图，先判断是不是想去某个地方再搜索。
- 相关说法分数偏低，就把这种说法写进点位描述。

## 测试

```bash
PYTHONPATH=src/semantic_map_ros python3 -m pytest src/semantic_map_ros/test   # 不需要模型和 Qdrant
bash tests/run_sim_e2e.sh                                                   # 全流程仿真（见仓库 README）
```
