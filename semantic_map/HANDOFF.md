# 交接说明（给接手的 Claude 会话 / 开发者）

## 目标

G1 听到一句话（如"我想喝水"）→ 找到地图上的目标位姿 `(x, y, yaw)` → 以后交给导航让机器人走过去。
参考：<https://github.com/yuniko-software/bge-m3-qdrant-sample>

## 已定下的决策

- 向量模型 BGE-M3，**只用 dense 向量**（1024 维，余弦距离）。地点少、文本短，不做示例里的 sparse/ColBERT 混合检索。
- 只把 title + description 送进模型，坐标不参与；整条记录（含坐标）存进 Qdrant 的 payload。
- Qdrant **用 Docker 跑**（用户明确选择，不用 qdrant-client 本地模式）。
- 坐标格式：地图坐标系 `map` 下的 `x, y`（米）+ `yaw`（弧度）。
- 查询分数低于 `SCORE_THRESHOLD`（config.py，初始 0.45）返回 `found: false`，机器人不动。
- 开发机：Ubuntu 笔记本，Intel Core Ultra 7 155H，无 NVIDIA 显卡 → CPU 版 torch，`USE_FP16=0`。
- 之后部署到 Jetson Thor：CUDA 版 torch、`USE_FP16=1`，代码不改。
- 用户是初学者：解释用中文、通俗、一步一步。

## 已验证 / 未验证

已在云端容器验证：
- Docker 版 Qdrant 启动、建库（8 个点，1024 维 + payload）、查询、payload 取坐标、JSON 输出、数据校验报错、重复建库。
- `setup.sh` 第 1–2 步（环境检查、启动 Qdrant）。

**未验证**（云端连不上 HuggingFace 和 download.pytorch.org）：
- 真实 BGE-M3 的检索效果，以及 0.45 阈值是否合适。
- `setup.sh` 第 3–6 步（装 torch、下载模型、建库、测试查询）。

## 下一步（按顺序）

1. 在用户电脑上运行 `bash setup.sh`，修掉遇到的报错（国内网络：Docker 镜像加速、`HF_ENDPOINT=https://hf-mirror.com`）。
2. 用真实模型测相关、无关的句子各十几句，看分数分布，调整 `SCORE_THRESHOLD`。
3. 问清 G1 用的导航/定位系统（ROS2 + Nav2？宇树自带？），然后：
   - 写标注工具：机器人站到某处 → 输入 title/description → 自动读取当前位姿，追加到 `data/semantic_map.json`；
   - 写导航对接：`search()` 结果 → 目标位姿（Nav2 用 `PoseStamped`，`z = sin(yaw/2)`，`w = cos(yaw/2)`）→ 发给导航。
4. 用 G1 建图后标注的真实坐标，替换 `data/semantic_map.json` 里编造的 8 个示例点。
5. 部署到 Thor（见 README 最后一节）。
