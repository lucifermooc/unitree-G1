# 语义地图完整数据流（从网页上点一个点，到机器人走过去）

本文按**现在代码里真实的实现**逐步写出每一步的输入、处理和输出数据。
文中文件路径都相对 `lightning_ws/src/`。

## 先纠正一个说法：现在已经没有 `semantic_map.json` 了

最早的方案是手写一个 `semantic_map.json`，里面记录每个地点的名称、描述和坐标 `{id, title, description, pose:{x,y,yaw}}`。
接入 G1 后，这个文件被**机器人原有的点位数据库**代替了：

| 早期方案 | 现在的实现 | 原因 |
|---|---|---|
| 手写 `semantic_map.json` | 数据库 `~/maps/db.sqlite` 的 `waypoint_node` 表 | 点位本来就存在数据库里，导航和巡逻也用它，不需要再维护一份 |
| `title` | `name` | 沿用原有点位字段 |
| `description` | `description` | 语义地图新加的可选字段，后端原样存储，不改后端代码 |
| `pose: {x, y, yaw}` | `position {x,y,z}` + `orientation {x,y,z,w}`（四元数） | 沿用原有点位格式，ROS 导航用的就是四元数 |
| `id` 手写 | 数据库自增 `id` | 自动生成，不会重复 |

所以"`semantic_map.json` 怎么来的"，现在的答案是：**在网页地图上拖一下、填名称和描述、点保存**，数据就进了数据库。
语义地图节点每次搜索前从数据库读出来，自动建库。

---

## 完整数据走向图

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 阶段 A：标注地点（人在网页上操作，每个地点一次）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[A1] 浏览器：在地图上按下鼠标、拖动、松开
     输入：屏幕像素 按下点 (sx, sy)=(412, 230)，松开点 (452, 190)
     处理：g1_web/www/app.js  s2w() 像素→地图米坐标；dragYaw() 拖动方向→朝向
     输出：{x: 3.2, y: -1.5, yaw: 0.7854}                             （JS 对象，只在浏览器内存里）
        │
        ▼
[A2] 浏览器：右侧表单填写名称和描述，点"保存"
     输入：name="茶水间"  description="有饮水机和咖啡机，可以接水、喝水、泡茶"
     处理：app.js  pointData()：yaw→四元数，拼成 JSON，再 JSON.stringify 变成字符串
     输出：字符串 data =
           '{"position":{"x":3.2,"y":-1.5,"z":0},
             "orientation":{"x":0,"y":0,"z":0.3827,"w":0.9239},
             "name":"茶水间","description":"有饮水机和咖啡机，可以接水、喝水、泡茶"}'
        │
        ▼  WebSocket  ws://<Thor IP>:9090   （rosbridge 协议，JSON 文本帧）
[A3] 浏览器 → rosbridge
     发送：{"op":"call_service","service":"/add_point",
            "type":"aid_robot_msgs/srv/OperationAdd",
            "args":{"map_id":2,"frame_id":"map","data":"<上面的字符串>","data_type":"waypoint_node"}}
        │
        ▼  rosbridge 把 JSON 转成 ROS2 服务请求
[A4] map_manager_server（原有后端，aid_robot_py/map_manager_server.py add_data_callback）
     处理：INSERT INTO waypoint_node(map_id, point_list, frame_id, create_timestamp, edit_timestamp, creator_id)
     输出：~/maps/db.sqlite 里新增一行
           ┌────┬────────┬──────────────────────────────────────────┬──────────┬─────┐
           │ id │ map_id │ point_list（TEXT，原样存 A2 的字符串）   │ frame_id │ ... │
           ├────┼────────┼──────────────────────────────────────────┼──────────┼─────┤
           │ 7  │ 2      │ {"position":{...},"orientation":{...},   │ map      │     │
           │    │        │  "name":"茶水间","description":"..."}    │          │     │
           └────┴────────┴──────────────────────────────────────────┴──────────┴─────┘
     返回：{"success": true, "message": "ok"}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 阶段 B：建库（语义地图节点自动做；点位有变化时的下一次搜索触发）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[B1] semantic_map_server → /get_current_map_id
     输出：{success: true, map_id: 2}                    （当前正在用哪张地图）
        │
        ▼
[B2] semantic_map_server → /get_map_point_list  请求 {map_id: 2, data_type: "waypoint_node"}
     map_manager_server 执行 SELECT id, frame_id, point_list FROM waypoint_node WHERE map_id=2
     输出 message（字符串，注意 point_list 仍然是字符串，嵌套了一层 JSON）：
       '[{"id": 7, "frame_id": "map", "point_list": "{\"position\": {\"x\": 3.2, ...}, \"name\": \"茶水间\", ...}"},
         {"id": 8, "frame_id": "map", "point_list": "{... \"name\": \"充电桩\", \"description\": \"机器人没电时回来充电\" ...}"},
         ...]'
        │
        ▼
[B3] core.py  parse_point_rows()：两层 JSON 解开，整理成统一的"地点"字典；没有名称或格式坏的行跳过
     输出 places（Python 列表）：
       [{"id": 7, "name": "茶水间", "description": "有饮水机和咖啡机，可以接水、喝水、泡茶",
         "frame_id": "map", "x": 3.2, "y": -1.5, "z": 0.0,
         "orientation": {"x": 0.0, "y": 0.0, "z": 0.3827, "w": 0.9239}},
        {"id": 8, "name": "充电桩", ...}, ...]
        │
        ▼
[B4] core.py  signature()：用 (id, 文本, x, y, 朝向) 算 sha1 指纹，和上次建库时的指纹比较
     相同且 collection 存在 → 跳过 B5~B8，直接进入阶段 C
     不同（有增删改）→ 重建
        │
        ├──────────────────────────────── 文本这一路 ─────────────────────────────────┐
        │                                                                              ▼
        │  [B5] core.py  place_text()：只取名称和描述，坐标不要
        │       输入：places[i]
        │       输出：["茶水间。有饮水机和咖啡机，可以接水、喝水、泡茶",
        │              "充电桩。机器人没电时回来充电", ...]          （没有描述时就只有名称）
        │                                                                              │
        │                                                                              ▼
        │  [B6] embedder.py  Embedder.encode() → BGE-M3 模型（BAAI/bge-m3）
        │       分词 → 神经网络 → 每句话输出一个 1024 维 dense 向量（已归一化，长度=1）
        │       输出：[[0.0213, -0.0487, 0.0071, ... 共 1024 个小数],      ← 茶水间
        │              [-0.0132, 0.0356, 0.0419, ... 共 1024 个小数], ...] ← 充电桩
        │                                                                              │
        ├──────────────────────────────── 坐标这一路 ─────────────────────────────────┤
        │                                                                              │
        │  [B7] payload = places[i] 整条原样（含 id、名称、描述、坐标、朝向）         │
        │       坐标不经过模型，原样跟着向量一起存                                     │
        │                                                                              ▼
        └─────────────────────────────────────────────────────────────────────▶ [B8] core.py  SemanticIndex.rebuild()
     Qdrant（Docker 容器，端口 6333）
       ① delete_collection("semantic_map_2")                     删掉这张地图的旧库
       ② create_collection("semantic_map_2", size=1024, distance=COSINE)
       ③ upsert：每个地点一个 Point = id + vector + payload
     Qdrant 里存下的样子：
       collection "semantic_map_2"
       ┌──────┬────────────────────────────────┬────────────────────────────────────────────────┐
       │ id   │ vector（1024 维）              │ payload（JSON）                                 │
       ├──────┼────────────────────────────────┼────────────────────────────────────────────────┤
       │ 7    │ [0.0213, -0.0487, ...]         │ {"id":7,"name":"茶水间","description":"...",    │
       │      │                                │  "frame_id":"map","x":3.2,"y":-1.5,"z":0.0,     │
       │      │                                │  "orientation":{"x":0,"y":0,"z":0.3827,"w":0.9239}} │
       │ 8    │ [-0.0132, 0.0356, ...]         │ {"id":8,"name":"充电桩", ...}                   │
       └──────┴────────────────────────────────┴────────────────────────────────────────────────┘
     数据文件落在 ~/maps/qdrant_storage（docker-compose.yml 挂载），重启不丢

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 阶段 C：查询（每说一句话一次）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[C1] 一句话从三个入口之一进来（data 可以是普通文本，也可以是 {"text": "...", "source": "谁发的"}）
     ① 网页"语义地图"输入"我想喝水"，点"搜索"/"去那里"
        → 服务 /semantic_map/search 或 /semantic_map/go，data='{"text":"我想喝水","source":"web"}'
     ② 其他程序调同样的服务，例如 data='{"text":"我想喝水","source":"asr"}'
     ③ ASR 往话题 /semantic_map/text_in（std_msgs/String）发 data="我想喝水"
        （只管发不等结果；默认只搜索，text_in_action=go 时找到就导航）
        │
        ▼
[C1.5] semantic_map_server  _handle()：三个入口都走这里
     core.parse_request()：拆出 原文 raw 和 来源 source（没带 source 就记成入口名 search / go / topic）
     core.inspect_text()：检查原文 → 问题列表，并只去掉不可见字符和首尾空白 → 送进模型的 text
       例：raw=" 我想\u200b喝水\n" → text="我想喝水"，issues=["首尾有空白：换行、空格", "含不可见字符：零宽空格"]
     清理后为空 → 出错 "empty query"（照样留调试记录）
        │
        ▼
[C2] semantic_map_server  _search()：先走一遍 B1~B4（点位没变就不重建，几毫秒）
        │
        ▼
[C3] embedder.py  encode(["我想喝水"])  → 同一个 BGE-M3 模型
     输出：查询向量 [0.0188, -0.0402, 0.0115, ... 共 1024 个小数]
        │
        ▼
[C4] Qdrant  query_points("semantic_map_2", query=查询向量, limit=3, with_payload=True)
     Qdrant 计算查询向量和库里每个向量的余弦相似度，按分数从高到低取前 3 个，连同 payload 返回
     输出：[ (id=7, score=0.7312, payload={茶水间...}),
             (id=8, score=0.4105, payload={充电桩...}),
             (id=3, score=0.3876, payload={前台...}) ]
        │
        ▼
[C5] core.py  SemanticIndex.search()：从 payload 取出坐标，四元数算回 yaw，判断阈值
     第一名分数 0.7312 ≥ score_threshold 0.52 → found=true；否则 found=false、best=null
     输出 JSON（作为服务应答的 message 字符串返回给调用方；话题入口没有应答）：
       {"query": "我想喝水", "found": true,
        "best": {"id": 7, "name": "茶水间", "description": "有饮水机和咖啡机，可以接水、喝水、泡茶",
                 "frame_id": "map", "x": 3.2, "y": -1.5, "z": 0.0, "yaw": 0.7854,
                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.3827, "w": 0.9239}, "score": 0.7312},
        "candidates": [ {...茶水间...}, {...充电桩, "score": 0.4105}, {...前台, "score": 0.3876} ],
        "map_id": 2, "rebuilt": false, "threshold": 0.52, "source": "web", "issues": []}
        │
        ├── 用的是 /semantic_map/search：到此结束，网页显示结果并在地图上高亮茶水间
        │
        ▼   用的是 /semantic_map/go 且 found=true：
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 阶段 D：导航
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[D1] semantic_map_server  _navigate()：如果 /task_status 显示正在执行或暂停（status 1/4），
     先调 /patrol_control {cmd: "cancel"}，等 0.5 秒（waypoint_manage 有任务时会忽略新目标）
        │
        ▼
[D2] _goal()：best → geometry_msgs/PoseStamped，发布到 /nav_to_pose
       header.frame_id = "map"，header.stamp = 当前时间
       pose.position    = {x: 3.2, y: -1.5, z: 0.0}
       pose.orientation = {x: 0, y: 0, z: 0.3827, w: 0.9239}
     应答 message 里多一个字段 "navigating": true
        │
        ▼
[D3] waypoint_manage（原有后端）nav_pose_callback：task_status 设为执行中，
     把 PoseStamped 包成 Nav2 NavigateToPose 动作目标发给 Nav2
        │
        ▼
[D4] Nav2 规划路径、控制机器人走到 (3.2, -1.5)，朝向 45°
     过程中 waypoint_manage 发布 /task_status，网页订阅后显示"执行中 / 已完成 / 失败"

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 阶段 E：调试记录（每次请求都有，成功失败都记；在 C1.5 之后、应答之前写）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[E1] _handle() 的 finally：拼一条记录
       {"time", "entry", "source", "raw"（原文一字不改）, "raw_visible"（␠ \n ⟨U+200B⟩ 标出看不见的字符）,
        "length", "issues", "text", "go", "ok"/"error", "map_id", "found", "best", "threshold",
        "candidates": [{id, name, score} × 3], "rebuilt", "navigating", "elapsed_ms"}
        │
        ├──▶ 话题 /semantic_map/debug（std_msgs/String，内容是这条 JSON）→ 网页调试面板实时显示
        ├──▶ 内存里最近 200 条 → 服务 /semantic_map/history（网页打开/刷新时补齐）
        └──▶ 文件 ~/maps/semantic_map_log.jsonl 追加一行（超过 20 MB 改名 .1 再重写）
     原文有问题时，节点日志还会打一行 WARN：input from asr has issues: [...] raw=␠我想⟨U+200B⟩喝水\n

     怎么判断是谁的问题：
       raw / issues 就不对（错字、多了空格换行、乱码）            → 发送方（ASR）的问题
       raw 没问题，但最高分低于阈值，或第一名不是想要的地点    → 点位描述或阈值的问题
       ok=false                                               → 看 error（没选地图、后端服务没起来等）
```

---

## 每一步的细节

### A1 屏幕像素 → 地图坐标（米）

网页把地图画在 canvas 上。地图参数来自 `/get_map_image` 返回的 `OccupancyGrid.info`：
`resolution`（每格多少米，通常 0.05）、`origin`（左下角那一格的地图坐标 `ox, oy`）、`height`（格数）。
另外网页自己记着缩放 `view.scale` 和平移 `view.ox, view.oy`。

`app.js` 里的 `s2w()`：

```
x = ox + (sx - view.ox) / view.scale × resolution
y = oy + (height - (sy - view.oy) / view.scale) × resolution      ← 屏幕 y 向下，地图 y 向上，所以要翻转
```

朝向 `dragYaw()`：从按下点到松开点的方向，`yaw = atan2(-dy, dx)`（同样因为屏幕 y 向下取负号）。
拖动距离不到 8 像素算"点一下"，朝向取 0（朝地图 x 正方向）。
结果保留 3 位小数（毫米），yaw 保留 4 位（弧度）。

也可以点"当前位置"，直接用 `/base_link_pose` 里机器人现在的坐标和朝向。

### A2 yaw → 四元数，拼成字符串

ROS 用四元数表示朝向。平面上只绕 z 轴转，所以：

```
x = 0, y = 0, z = sin(yaw / 2), w = cos(yaw / 2)
yaw = 0.7854（45°） → z = 0.3827, w = 0.9239
```

`pointData()` 用 `JSON.stringify` 把整个对象变成**字符串**。
必须是字符串，因为 `OperationAdd.srv` 里 `data` 的类型是 `string`，传对象会被 rosbridge 拒绝。

### A4 数据库表结构（`aid_robot_py/db/db.sql`，原有）

```sql
CREATE TABLE waypoint_node (
    id               integer primary key autoincrement,   -- 就是语义库里 Point 的 id
    map_id           integer,                              -- 属于哪张地图
    point_list       text,                                 -- A2 的 JSON 字符串，原样存
    frame_id         text,                                 -- "map"
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int                                   -- 0
);
```

后端不解析 `point_list`，所以加一个 `description` 字段完全不用改后端。
修改点位走 `/update_point`，删除走 `/delete_point`，都会改这张表，B4 的指纹随之变化。

### B3 解析：为什么是"两层 JSON"

`/get_map_point_list` 的 `message` 是 `json.dumps(整张查询结果)`，而每行的 `point_list` 本来就是字符串，
所以要先 `json.loads(message)` 得到行列表，再对每行 `json.loads(row["point_list"])`。
解析后没有 `name` 的点跳过；`description` 没有就当空字符串。

### B4 指纹：什么时候重建

`signature()` 把每个点的 `(id, 送进模型的文本, x, y, 朝向 z, 朝向 w)` 按 id 排序后做 sha1。
只要增删了点、改了名称/描述/坐标，指纹就变，下一次搜索自动重建；没变就直接搜。
手动重建可调 `/semantic_map/rebuild`。节点重启后内存里的指纹没了，第一次搜索会重建一次。

一张地图一个 collection，名字是 `semantic_map_<map_id>`，所以换地图不会串库。

### B5 文本：为什么坐标要剔除

`place_text()` 返回 `"名称。描述"`（没有描述就只有名称）。
模型只懂语言，`3.2, -1.5` 这种数字对"喝水"和"茶水间"像不像没有帮助，反而会干扰分数。
坐标不需要"被理解"，只需要"被找回来"，所以放在 payload 里原样存取。

**描述写法建议**：写上人会怎么说这个地方的需求，例如"接水、喝水、泡茶、热饭"，比只写"茶水间"更容易被"我渴了"搜到。

### B6 BGE-M3 怎么把一句话变成 1024 个数

`embedder.py` 调 FlagEmbedding 库的 `BGEM3FlagModel.encode()`，参数：

| 参数 | 值 | 含义 |
|---|---|---|
| `batch_size` | 16 | 一次送 16 句话进模型，批量更快 |
| `max_length` | 512 | 一句话最多 512 个词元，超出截断（地点描述远远用不到） |
| `return_dense` | True | 只要 dense 向量（1024 维） |
| `return_sparse` / `return_colbert_vecs` | False | 关键词权重、逐词向量这两种不用 |
| `use_fp16` | 电脑 False / Thor True | 半精度，GPU 上快一倍，CPU 上不要开 |

模型内部：句子先被切成词元（"茶水间" → 若干个编号），经过 24 层 Transformer，
最后取第一个位置（CLS）的输出，归一化成长度为 1 的 1024 维向量。
意思相近的句子，向量方向也相近，这是模型在海量句子对上训练出来的能力。
模型第一次用时下载约 2.3 GB，存在 `~/.cache/huggingface`，之后只从本地加载；节点启动时在后台预加载。

### B7 / B8 payload 和 Point

Qdrant 的一个 Point 由三部分组成：

- `id`：直接用数据库的点位 id（Qdrant 要求是无符号整数或 UUID，自增 id 正好满足），这样搜到后能对上数据库里是哪个点。
- `vector`：B6 的 1024 个数。
- `payload`：B3 整理出的那个字典原样放进去。Qdrant 不看 payload 的内容，只负责存和原样返回。

`rebuild()` 每次都是"删掉整个 collection 再重建"，而不是逐个修改，逻辑最简单，
几十到几百个点重建一次也只要一两秒。
`distance=COSINE` 表示按余弦相似度比较：两个向量夹角越小，分数越接近 1。

### C3 / C4 查询为什么要用同一个模型

查询句和地点文本必须用**同一个模型**变成向量，才在同一个"坐标系"里，才能比较。
Qdrant 算出 `score = 查询向量 · 地点向量`（都已归一化，点积就是余弦相似度），范围大致 -1 到 1，实际多在 0.3~0.8。

### C5 阈值

最高分低于 `score_threshold`（默认 0.52，在 `config/semantic_map.yaml` 或启动参数里改）就返回 `found: false`，
防止说一句无关的话（例如"今天股票涨了吗"）机器人也跑去某个地方。
阈值要用真实点位和真实说法来调：

```bash
python3 -m semantic_map_ros.evaluate --db ~/maps/db.sqlite --queries config/eval_queries_example.json
```

它会列出每句话的得分并推荐一个阈值。每次搜索的得分也会打印在节点日志里：
`search '我想喝水' map=2 points=8 rebuilt=False found=True [茶水间:0.731, 充电桩:0.411, 前台:0.388]`。

### D 导航

`/semantic_map/go` 不直接控制机器人，只是像原来前端点"导航到点位"一样，往 `/nav_to_pose` 发一个 `PoseStamped`，
后面的执行、状态上报全部由原有的 `waypoint_manage` 和 Nav2 完成。
导航前机器人需要处于巡逻模式（网页点"去这里"时会先调 `/mode_set` 切过去）。

---

## 各数据存在哪里

| 数据 | 位置 | 谁写 | 谁读 |
|---|---|---|---|
| 点位（名称、描述、坐标） | Thor `~/maps/db.sqlite` 的 `waypoint_node` 表 | 网页 → `/add_point` 等 → map_manager_server | 网页、waypoint_manage、semantic_map_server |
| 向量 + payload | Qdrant collection `semantic_map_<map_id>`，文件在 `~/maps/qdrant_storage` | semantic_map_server | semantic_map_server |
| BGE-M3 模型文件 | `~/.cache/huggingface/hub/models--BAAI--bge-m3` | 第一次加载时下载 | semantic_map_server |
| 指纹 | semantic_map_server 进程内存 | semantic_map_server | semantic_map_server |
| 调试记录 | `~/maps/semantic_map_log.jsonl` + 进程内存最近 200 条 | semantic_map_server | 网页、`ros2 topic echo /semantic_map/debug`、人工查看 |

数据库是唯一的"真数据"；Qdrant 里的东西随时可以删掉，下一次搜索会从数据库重新建出来。

---

## 对应的代码位置

| 步骤 | 文件 | 函数 |
|---|---|---|
| A1 | `g1_web/www/app.js` | `s2w`、`dragYaw`、`newPoint` |
| A2~A3 | `g1_web/www/app.js` | `pointData`、`savePoint`、`call` |
| A4 | `aid_robot_py/aid_robot_py/map_manager_server.py`（原有） | `add_data_callback` |
| B1~B2 | `semantic_map_ros/semantic_map_ros/semantic_map_server.py` | `_current_places` |
| B3 | `semantic_map_ros/semantic_map_ros/core.py` | `parse_point_rows` |
| B4 | `core.py` / `semantic_map_server.py` | `signature`、`is_fresh`、`_ensure_index` |
| B5 | `core.py` | `place_text` |
| B6、C3 | `semantic_map_ros/semantic_map_ros/embedder.py` | `Embedder.encode` |
| B7~B8 | `core.py` | `SemanticIndex.rebuild` |
| C4~C5 | `core.py` | `SemanticIndex.search`、`yaw_of` |
| C1、结果显示 | `g1_web/www/app.js` | `semantic`、`renderSemantic`、`sendAsrTest` |
| C1.5 | `semantic_map_server.py` / `core.py` | `_handle`、`_on_text_in`、`parse_request`、`inspect_text` |
| D1~D2 | `semantic_map_server.py` | `_navigate`、`_goal` |
| E1 | `semantic_map_server.py` / `debug_log.py` / `app.js` | `_handle`、`DebugLog`、`visible`、`renderSemLog` |
| D3 | `aid_robot_py/aid_robot_py/waypoint_manage.py`（原有） | `nav_pose_callback`、`send_next_goal` |

说明：图中向量的具体数值和相似度分数是示意，实际数值以模型输出为准；其余字段名、格式和流程都与代码一致。
