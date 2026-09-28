# 交接说明（给接手的 Claude 会话 / 开发者）

## 目标

G1 听到一句话（如"我想喝水"）→ 找到地图上的目标位姿 `(x, y, yaw)` → 以后交给导航让机器人走过去。
参考：<https://github.com/yuniko-software/bge-m3-qdrant-sample>

## 已定下的决策

- 向量模型 BGE-M3，**只用 dense 向量**（1024 维，余弦距离）。地点少、文本短，不做示例里的 sparse/ColBERT 混合检索。
- 只把 title + description 送进模型，坐标不参与；整条记录（含坐标）存进 Qdrant 的 payload。
- Qdrant **用 Docker 跑**（用户明确选择，不用 qdrant-client 本地模式）。
- 坐标格式：地图坐标系 `map` 下的 `x, y`（米）+ `yaw`（弧度）。
- 查询分数低于 `SCORE_THRESHOLD`（config.py，实测定为 0.52）返回 `found: false`，机器人不动。
- 开发机：Ubuntu 笔记本，Intel Core Ultra 7 155H，无 NVIDIA 显卡 → CPU 版 torch，`USE_FP16=0`。
- 之后部署到 Jetson Thor：CUDA 版 torch、`USE_FP16=1`，代码不改。
- 用户是初学者：解释用中文、通俗、一步一步。

## 已验证 / 未验证

已在云端容器（CPU，torch 2.14.0+cpu，FlagEmbedding 1.4.2，transformers 5.17.0，qdrant-client 1.19.1，Qdrant 1.19.1）用真实 BGE-M3 验证：
- `setup.sh` 全部 6 步跑通。
- `evaluate.py`：28 句相关的话第一名全部正确；无关句子最高分 0.565（"你好"），其余 ≤0.505。阈值定为 0.52，只放过"你好"，不误拒相关句子。

未验证：
- 用户笔记本上的安装过程（国内网络：Docker 镜像加速、HF 镜像）。
- 真实地点数据的效果（示例的 8 个地点和坐标都是编的）。

## 仿真全流程（sim/，已验证）

TurtleBot3 仿真里跑通：Cartographer 建图 → Cartographer 纯定位 + Nav2 → `semantic_map/semantic_nav.py`（一句话 → 语义地图 → Nav2 目标）。
`bash sim/2_semantic_nav_test.sh`：9/9 通过，到达误差 ≤ 0.18 m，定位误差 ≤ 0.03 m。细节见 `sim/README.md`。

## 下一步（按顺序）

1. 在用户电脑上运行 `bash setup.sh`，修掉遇到的报错（国内网络：Docker 镜像加速、`HF_ENDPOINT=https://hf-mirror.com`）。
2. 换成真实地点后，同步改 `data/eval_queries.json`，运行 `python evaluate.py` 重新确定 `SCORE_THRESHOLD`。
3. 问清 G1 用的导航/定位系统（ROS2 + Nav2？宇树自带？），然后：
   - 标注地点：用 `label_tool.py` 在网页地图上点选（已完成、已在仿真验证）；
   - 导航对接：如果 G1 用 Nav2，直接用 `semantic_map/semantic_nav.py`（仿真已验证）；否则参考其中的 `goal_to_pose()` 适配。
4. 用 G1 建图后标注的真实坐标，替换 `data/semantic_map.json` 里编造的 8 个示例点。
5. 部署到 Thor（见 README 最后一节）。
