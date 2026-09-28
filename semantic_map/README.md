# 语义地图：一句话 → 地图坐标（BGE-M3 + Qdrant）

让 G1 听懂"我想喝水""去充电"，并找到地图上对应地点的坐标 `(x, y, yaw)`。
参考了 [yuniko-software/bge-m3-qdrant-sample](https://github.com/yuniko-software/bge-m3-qdrant-sample)，把那个示例里的"商品"换成了"地图上的地点"。

## 原理（先看懂这张图）

```
【建库：离线跑一次  build_index.py】

data/semantic_map.json
  每个地点 = { id, title, description, pose:{x, y, yaw} }
        │
        ├── 文本 = title + description   （坐标剔除，不进模型）
        │        │
        │        ▼
        │     BGE-M3 ──> 1024 维向量
        │                      │
        └── 整条记录（含坐标）── │ ──> 作为 payload
                               ▼
  Qdrant 里的一个点(Point) = [ id | payload(JSON，里面有坐标) | vector(1024维) ]

【查询：每次说话都跑  query.py】

"我想喝水" ──> BGE-M3 ──> 查询向量 ──> Qdrant 按余弦相似度找最像的点
          ──> 从该点的 payload 取出坐标 ──> 输出 JSON：{title, x, y, yaw, score}
```

要点：
- **坐标不进模型**。模型只理解文字意思，坐标数字放进去只会干扰匹配；坐标原样存在 payload 里，搜到了直接拿出来。
- **JSON 出现在两处**：输入的地点数据文件是 JSON；Qdrant 的 payload 和查询结果也是 JSON（后面直接给导航用）。
- **建库和查询用同一个模型**，向量才在同一个空间里，才能比较相似度。
- 示例仓库用了 dense + sparse + ColBERT 三种向量。地点只有几十上百条、文字很短，这里**只用 dense 向量**，简单且够用。

## 文件说明

| 文件 | 作用 |
|---|---|
| `docker-compose.yml` | 启动 Qdrant 向量数据库 |
| `requirements.txt` | Python 依赖 |
| `config.py` | 所有配置（地址、模型、阈值……） |
| `data/semantic_map.json` | 地点数据，**换成你自己的** |
| `embedder.py` | BGE-M3：文本 → 向量 |
| `build_index.py` | 建库 |
| `query.py` | 查询（命令行 / 被其他程序 import） |

## 第一次运行（Ubuntu 笔记本，只用 CPU）

所有命令都在 `semantic_map/` 目录下执行。

**1. 启动 Qdrant**（需要先装好 Docker）

```bash
cd semantic_map
docker compose up -d
```

浏览器打开 <http://localhost:6333/dashboard>，能看到 Qdrant 的网页就说明启动成功了。

**2. 安装 Python 依赖**（Python 3.10 及以上）

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu   # CPU 版，比 CUDA 版小很多
pip install -r requirements.txt
```

**3. 建库**

```bash
# 国内网络下载 HuggingFace 模型慢/连不上时先执行这一行
export HF_ENDPOINT=https://hf-mirror.com

python build_index.py
```

第一次会下载 BGE-M3 模型（约 2.3GB），之后就不用再下了。成功时会输出：

```
读取到 8 个地点：.../data/semantic_map.json
示例文本（送进模型的内容）： 前台。公司入口处的接待台，访客登记、取快递、问路都在这里
已写入 8 个地点到 Qdrant collection 'semantic_map'
```

这时去 dashboard → Collections → `semantic_map`，能看到 8 个点，点开能看到 payload 里的坐标。

**4. 查询**

```bash
python query.py "我想喝水"
```

输出示例（分数以实际为准）：

```json
{
  "query": "我想喝水",
  "found": true,
  "best": {"id": 2, "title": "茶水间", "x": 3.2, "y": -1.5, "yaw": 1.57, "frame_id": "map", "score": 0.68},
  "candidates": [ ...前 3 名，调试用... ]
}
```

不带参数运行 `python query.py` 进入交互模式，模型只加载一次，可以连续试很多句话。

建议测试这几句：

| 输入 | 期望 |
|---|---|
| 我想喝水 | 茶水间 |
| 机器人没电了 | 充电桩 |
| 一会儿要开会 | 会议室 |
| 今天股票涨了吗 | `found: false`（没有相关地点） |

## 换成你自己的地图

1. 在 G1 上建好地图后，把每个想让机器人去的地方记下坐标（map 坐标系下的 x、y，单位米；朝向 yaw，单位弧度）。
2. 按下面的格式写进 `data/semantic_map.json`：

```json
{
  "id": 9,
  "title": "打印机",
  "description": "打印、复印、扫描文件的地方",
  "pose": {"x": 1.5, "y": -3.0, "yaw": 0.0},
  "frame_id": "map"
}
```

   - `id`：整数，不能重复。
   - `description`：**写成人会怎么说**。比如充电桩写上"没电了""电量低"，用户说这些话时才更容易匹配到。
3. 重新运行 `python build_index.py`（会自动删掉旧库重建）。

## 调阈值

`config.py` 里的 `SCORE_THRESHOLD`（默认 0.45）：最相似地点的分数低于它，就返回 `found: false`，机器人不动。

- 相关的话也经常 `found: false` → 调低一点
- 不相关的话也能找到地点 → 调高一点

最好的办法：准备 20 句相关的话、10 句不相关的话，看分数分布，然后取中间值。

## 在程序里调用（接导航）

```python
from query import search

result = search("去充电")
if result["found"]:
    goal = result["best"]
    # 把 goal["x"], goal["y"], goal["yaw"] 发给 G1 的导航模块
else:
    print("没听懂要去哪里")
```

## 以后部署到 Jetson Thor

代码不用改，只改环境：

1. Qdrant：官方镜像支持 ARM64，同样 `docker compose up -d`。
2. torch：装 NVIDIA 为 Jetson 提供的 CUDA 版 PyTorch（不要用上面的 CPU 版命令）。
3. 用 GPU 半精度加速：`export USE_FP16=1`。
4. 把笔记本上下载好的模型目录（`~/.cache/huggingface/hub/models--BAAI--bge-m3`）拷到 Thor 同一位置，机器人上就不用联网下载了。
5. 如果 Qdrant 跑在别的机器上：`export QDRANT_HOST=那台机器的IP`。

## 常见问题

- **`Connection refused` / 连不上 6333**：Qdrant 没启动，执行 `docker compose up -d`，用 `docker ps` 确认。
- **`Collection semantic_map not found`**：还没建库，先跑 `python build_index.py`。
- **模型下载失败**：设置 `export HF_ENDPOINT=https://hf-mirror.com` 后重试。
- **查询慢**：CPU 上第一次加载模型要十几秒到几十秒；之后每次查询很快。用交互模式或在程序里 import `search`，模型只加载一次。
