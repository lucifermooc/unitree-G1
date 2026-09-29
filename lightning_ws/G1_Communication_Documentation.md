## G1 通信接口文档（前端 ↔ 机器人）

> 2026-09 版：逐条对照 `lightning_ws/src` 源码和仿真实测修订，修订内容见文末《附录 A 修订记录》。
> 新增《第五章 语义地图》。标 **⚠** 的地方是和旧文档不一样、或者使用时容易踩坑的地方。

### 第零章 连接方式

- 前端通过 **rosbridge** 和机器人通信：`ws://<机器人IP>:9090`（`robot.launch.py` 默认 `start_rosbridge:=true`）。
- 消息是 rosbridge v2 协议的 JSON：发布话题 `op: publish`，订阅 `op: subscribe`，调用服务 `op: call_service`。
- ⚠ 服务的 `args` 必须用**对象**：`{"action": "mapping"}`。旧文档很多地方写成数组 `[{"action": "mapping"}]`，
  rosbridge 会把数组当成"按字段顺序排列的值"，实测直接报错 `msg is not a primitive type, but a <class 'dict'>`。
- ⚠ **类型是 `string` 的字段，即使内容是 JSON，也必须传字符串**（先 `JSON.stringify`）。例如 `add_point` 的 `data`、
  `update_map` 的 `data`。传对象会被 rosbridge 拒绝。
- ⚠ 服务请求里**不能带服务定义里没有的字段**，否则 rosbridge 直接报错
  （例如 `Message type ... does not have a field patrol_count`）。
- 机器人上的 rosbridge 配置 `default_call_service_timeout: 0.0`（服务调用不超时）。
- 网页控制台：静态网页，不用放到机器人上。在电脑上 `cd lightning_ws/src/g1_web/www && python3 -m http.server 8080`，
  浏览器打开 `http://localhost:8080/?host=<机器人IP>`。

---

### 第一章 状态管理

#### 1.1 重定位 ⚠（机器人端暂未实现）

说明：把机器人在地图中的位置设为指定位姿（通常是建图起点 / 充电点）。

⚠ 当前代码里**没有节点处理重定位**：
- `/start_init_pose`：没有任何节点订阅。
- `/aid_init_pose`（`geometry_msgs/msg/PoseStamped`）：`robot_status_manager` 订阅后转发成 `/initialpose`，
  但 Lightning 定位**不订阅** `/initialpose`，所以不会生效。

前端（原海尔前端的 relocation 页面、新网页控制台）发的是下面两条消息，等机器人端实现后即可生效：

**Topic**: `/initialpose`　**Message Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`

````
{
  "op": "publish",
  "topic": "/initialpose",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      },
      "covariance": [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853891945200942]
    }
  }
}
````

**Topic**: `/start_init_pose`　**Message Type**: `geometry_msgs/msg/PoseStamped`（与上面同一个位姿，不带协方差）

````
{
  "op": "publish",
  "topic": "/start_init_pose",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
````

#### 1.2 机器人当前位置

说明：机器人在地图中的实时位姿（= TF `map → base_link`），由 `robot_pose_pub_node` 发布。

⚠ 只有定位或建图在运行时才有数据：`map → base_link` 超过一定时间没有更新就停发（空闲模式下收不到）。

**Topic**: `/base_link_pose`　**Message Type**: `geometry_msgs/msg/PoseStamped`

````
{
  "op": "subscribe",
  "topic": "/base_link_pose",
  "type": "geometry_msgs/msg/PoseStamped",
  "throttle_rate": 200,
  "queue_length": 1
}
````

**接收的消息**：

````
{
  "op": "publish",
  "topic": "/base_link_pose",
  "msg": {
    "header": {"stamp": {"sec": 1234567890, "nanosec": 123456789}, "frame_id": "map"},
    "pose": {
      "position": {"x": 1.234, "y": 5.678, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
````

朝向（弧度）：`yaw = atan2(2(w·z + x·y), 1 − 2(y² + z²))`。

#### 1.3 电量信息 ⚠

**Topic**: `/battery_state`（旧文档响应示例里写成了 `/battery_data`，实际话题名是 `/battery_state`）
**Message Type**: `sensor_msgs/msg/BatteryState`，由 `g1_nav_bridge/battery_state_bridge` 发布。

````
{
  "op": "subscribe",
  "topic": "/battery_state",
  "type": "sensor_msgs/msg/BatteryState",
  "throttle_rate": 1000,
  "compression": "cbor"
}
````

字段含义（按 `battery_state_bridge.cpp` 实际实现）：

| 字段 | 含义 |
|---|---|
| `percentage` | 电量，**0~1**（显示时 ×100） |
| `voltage` / `current` | 电压（V）/ 电流（A，放电为负） |
| `power_supply_status` | **判断是否充电看这个**：1 充电中，2 放电，3 未充电，4 已充满 |
| `charge` / `capacity` / `design_capacity` | ⚠ 恒为 **NaN**（原前端用 `charge > 0.1` 判断充电，在 G1 上永远判断不到） |
| `temperature` | 电池温度，读不到时为 NaN |

⚠ 消息里有 NaN，建议按上面加 `"compression": "cbor"` 订阅（CBOR 能表示 NaN）。

#### 1.4 机器人模式状态（新增）

说明：机器人当前的 SLAM 状态和控制模式，`robot_status_manager` 每秒发布一次。

**Topic**: `/robot_status`　**Message Type**: `std_msgs/msg/String`

````
{"op": "subscribe", "topic": "/robot_status", "type": "std_msgs/msg/String"}
````

`msg.data` 格式为 `"<slam状态>+<控制模式>"`，例如 `"localization+patrol"`：
- slam 状态：`idle`（空闲）、`mapping`（建图中）、`localization`（定位中，Nav2 同时在运行）
- 控制模式：`idle`、`patrol`（导航/巡逻）、`remote_control`（遥控）

---

### 第二章 地图管理

#### 2.1 地图位置点管理

位置点存放在地图数据库（`~/maps/db.sqlite`）的 `waypoint_node` 表，由 `map_manager_server` 管理。

⚠ 一个位置点的内容（`data` / `point_list`）是一个 **JSON 字符串**，格式：

````
{
  "position": {"x": 1.0, "y": 2.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "name": "茶水间",
  "description": "有饮水机和咖啡机，可以接水、喝水、泡茶"
}
````

`description` 是语义地图用的**可选字段**（见第五章），后端原样存储，不影响其它功能。

##### 2.1.1 新增位置点

**Service**: `/add_point`　**Service Type**: `aid_robot_msgs/srv/OperationAdd`

````
- 入参：
  - uint32 map_id     地图 id
  - string frame_id   固定 "map"
  - string data       位置点内容（⚠ JSON 字符串）
  - string data_type  固定 "waypoint_node"
- 返回：
  - bool success
  - string message    成功 "ok"；数据库出错 "error"
````

请求：

````
{
  "op": "call_service",
  "service": "/add_point",
  "args": {
    "map_id": 1,
    "frame_id": "map",
    "data": "{\"position\":{\"x\":1.0,\"y\":2.0,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0},\"name\":\"茶水间\",\"description\":\"可以接水、喝水\"}",
    "data_type": "waypoint_node"
  }
}
````

响应：

````
{"op": "service_response", "service": "/add_point", "values": {"success": true, "message": "ok"}, "result": true}
````

##### 2.1.2 删除位置点

**Service**: `/delete_point`　**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
{
  "op": "call_service",
  "service": "/delete_point",
  "args": {"id": 1, "data_type": "waypoint_node"}
}
````

响应：`{"success": true, "message": "ok"}`（出错为 `"error"`）。

##### 2.1.3 修改位置点

**Service**: `/update_point`　**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

⚠ `data` 是 JSON 字符串，会**整体替换**这个点原来的内容（要改名字、描述、坐标都要把完整内容传过去）。

````
{
  "op": "call_service",
  "service": "/update_point",
  "args": {
    "id": 1,
    "data": "{\"position\":{\"x\":1.5,\"y\":2.0,\"z\":0.0},\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.7071068,\"w\":0.7071068},\"name\":\"茶水间\",\"description\":\"可以接水、喝水、泡茶\"}",
    "data_type": "waypoint_node"
  }
}
````

响应：成功 `{"success": true, "message": "ok"}`；id 不存在 `{"success": false, "message": "data not found"}`。

##### 2.1.4 获取地图的位置点列表

**Service**: `/get_map_point_list`　**Service Type**: `aid_robot_msgs/srv/MapLinkedDataList`

````
{
  "op": "call_service",
  "service": "/get_map_point_list",
  "args": {"map_id": 1, "data_type": "waypoint_node"}
}
````

⚠ 响应的 `message` 是 **JSON 字符串**；解析后每一项的 `point_list` **又是一个 JSON 字符串**，需要再解析一次：

````
{
  "op": "service_response",
  "service": "/get_map_point_list",
  "values": {
    "success": true,
    "message": "[{\"id\": 1, \"frame_id\": \"map\", \"point_list\": \"{\\\"position\\\":{\\\"x\\\":1.0,\\\"y\\\":2.0,\\\"z\\\":0.0},\\\"orientation\\\":{\\\"x\\\":0,\\\"y\\\":0,\\\"z\\\":0,\\\"w\\\":1},\\\"name\\\":\\\"茶水间\\\"}\"}]"
  },
  "result": true
}
````

````js
const points = JSON.parse(res.message).map(row => ({ id: row.id, ...JSON.parse(row.point_list) }));
````

#### 2.2 地图管理

##### 2.2.1 开始建图

说明：切换到建图模式（`robot_status_manager` 停掉定位/导航，启动 Lightning 建图）。

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "mapping"}}
````

响应 ⚠（字段名是 `message`，不是旧文档的 `msg`）：

````
{"op": "service_response", "service": "/mode_set", "values": {"message": "ok"}, "result": true}
````

失败时 `message` 为 `"err"`。

##### 2.2.2 取消建图

说明：切换到空闲模式，停止建图，**不保存**。

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "idle"}}
````

响应：`{"message": "ok"}`。

##### 2.2.3 保存地图 ⚠（两步，旧文档缺第一步）

**第 1 步：保存建图结果到文件**

**Service**: `/aid_save_map`　**Service Type**: `aid_robot_msgs/srv/MapOperation`（`robot_status_manager` 提供）

说明：只能在建图模式下调用。调用 Lightning 的 `/lightning/save_map` 保存地图，然后停止建图，模式回到 idle。

````
- 入参：
  - string map_file_name  地图目录，⚠ 相对家目录：填 "/maps/<名字>" → 保存到 ~/maps/<名字>/
- 返回：
  - bool success
  - string message  失败原因："must change mode to mapping first"、"lightning map save failed"、
                    "failed to stop mapping after saving"
````

````
{"op": "call_service", "service": "/aid_save_map", "args": {"map_file_name": "/maps/1790604029000"}}
````

**第 2 步：写入地图数据库**

**Service**: `/add_map`　**Service Type**: ⚠ `aid_robot_msgs/srv/MapOperationAdd`（旧文档写成了 MapOperation）

````
- 入参：
  - string map_name  地图名字（不能与已有地图重名）
  - string map_file  ⚠ 与第 1 步的 map_file_name 相同（相对家目录，不是绝对路径）
- 返回：
  - bool success
  - string message  成功 "ok"；重名 "map name duplicated"；
                    目录里找不到 map.yaml 或 lightning/map.yaml 时 "map file not exists <路径>"；其它 "error"
````

````
{
  "op": "call_service",
  "service": "/add_map",
  "args": {"map_name": "一楼办公室", "map_file": "/maps/1790604029000"}
}
````

响应：`{"success": true, "message": "ok"}`。

##### 2.2.4 建图实时预览

**Topic**: `/map_base64`　**Message Type**: `nav_msgs/msg/OccupancyGrid`

由 `aid_robot_py/map_transform_node` 发布（`robot.launch.py` 启动）：订阅 Lightning 建图发布的原始栅格 `/map`，
转成 JPEG 图片后发布。（旧文档写的是 map_manager_server 输出，实际是 map_transform。）

````
{"op": "subscribe", "topic": "/map_base64", "type": "nav_msgs/msg/OccupancyGrid", "compression": "png"}
````

⚠ `msg.data` 不是栅格数据，而是 **JPEG 图片的 base64 字符串逐字节放进去**（与 2.2.15 `get_map_image` 相同）：

````js
const b64 = new TextDecoder().decode(new Uint8Array(msg.data));
img.src = "data:image/jpg;base64," + b64;
// 像素坐标 ↔ 地图坐标：u = (x - origin.x) / resolution；v = height - (y - origin.y) / resolution
````

图片颜色：障碍物黑色；空闲区 RGB(127,145,200)；未探索区 RGB(82,108,170)；占用概率居中的格子 RGB(100,125,185)。

##### 2.2.5 切换地图

**Service**: `/set_current_map_id`　**Service Type**: `aid_robot_msgs/srv/SetCurrentMap`

````
{"op": "call_service", "service": "/set_current_map_id", "args": {"id": 1}}
````

响应：`{"success": true}`（只有这一个字段）。

⚠ 这一步只改数据库里的"当前地图"。要让定位/导航真正用上这张图，接着调用
`/mode_set {"action": "localization"}`（`robot_status_manager` 会读当前地图并重启定位和 Nav2）。

##### 2.2.9 禁行线 ⚠（旧文档的 `/draw_no_go_lines` 不存在）

实际由两个服务配合完成（与原前端 editMap 页面一致）：

**① 保存到数据库**：**Service** `/set_forbidden`　**Type** `aid_robot_msgs/srv/ForbiddenSet`（`map_manager_server`）

每张地图只保存一组禁行线，再次调用会整体覆盖。

````
{
  "op": "call_service",
  "service": "/set_forbidden",
  "args": {
    "map_id": 1,
    "frame_id": "map",
    "lines": [{"start": {"x": 1.0, "y": 2.0, "z": 0.0}, "end": {"x": 3.0, "y": 4.0, "z": 0.0}}]
  }
}
````

响应：`{"success": true, "message": "Successfully added forbidden lines: 1"}`；地图不存在时 `success: false`。

**② 画到导航用的禁行地图**：**Service** `/aid_draw_forbidden_line`　**Type** `aid_robot_msgs/srv/DrawPicture`
（`forbidden_map_create_node`，`robot.launch.py` 的 `use_keepout:=true` 时启动，生成 `/keepout_filter_map` 给 Nav2）

````
{
  "op": "call_service",
  "service": "/aid_draw_forbidden_line",
  "args": {
    "frame_id": "map",
    "type": "line",
    "map_id": 1,
    "data": [{"start": {"x": 1.0, "y": 2.0, "z": 0.0}, "end": {"x": 3.0, "y": 4.0, "z": 0.0}}],
    "rectangle_array": []
  }
}
````

响应：`{"success": true, "message": "success"}`。⚠ 还没收到 `/map`（没在定位模式）时返回
`{"success": false, "message": "No map get."}`。

**③ 读取**：**Service** `/get_forbidden`　**Type** `aid_robot_msgs/srv/ForbiddenGet`，`args: {"map_id": 1}`，
返回 `{"success": true, "lines": [{"start": {...}, "end": {...}}], "message": "success"}`。

##### 2.2.10 橡皮擦

**Service**: `/map_editor`　**Service Type**: `aid_robot_msgs/srv/DrawPicture`（`editor_map_node`）

说明：把若干正方形区域写成指定值，并**直接改写地图文件**（第一次修改前自动备份 `map_back`）。

````
{
  "op": "call_service",
  "service": "/map_editor",
  "args": {
    "frame_id": "map",
    "type": "eraser",
    "map_id": 1,
    "data": [],
    "rectangle_array": [
      {"center_point": {"x": 0.0, "y": 0.0, "z": 0.0}, "side_length": 0.5, "grayscale": 0}
    ]
  }
}
````

- `grayscale` 是**占用值**：`0` 擦成空闲，`100` 画成障碍。
- 响应：`{"success": true, "message": "Map updated successfully"}`。
- ⚠ 改的是地图文件，已经在运行的定位/导航不会自动重新加载；需要重新 `/mode_set localization` 才生效。

##### 2.2.11 删除地图

**Service**: `/delete_map`　**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
{"op": "call_service", "service": "/delete_map", "args": {"id": 1, "data_type": "map"}}
````

响应：`{"success": true, "message": "ok"}`。会一并删除这张地图的位置点、禁行线、"当前地图"记录，
以及 `~/maps` 下的地图文件。

##### 2.2.12 更新地图（重命名）

**Service**: `/update_map`　**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

````
{
  "op": "call_service",
  "service": "/update_map",
  "args": {"id": 1, "data": "{\"name\": \"新名字\"}", "data_type": "map"}
}
````

- ⚠ `data` 是 JSON **字符串**，键是 map 表的列名；**只建议改 `name`**（后端会把键直接拼进 SQL）。
- 响应：成功 `{"success": true, "message": "ok"}`；id 不存在 `{"success": false, "message": "data not found"}`。

##### 2.2.13 获取地图列表

**Service**: `/get_map_list`　**Service Type**: `aid_robot_msgs/srv/MapList`

````
{"op": "call_service", "service": "/get_map_list", "args": {}}
````

响应：

````
{
  "op": "service_response",
  "service": "/get_map_list",
  "values": {"success": true, "map_list": "[{\"id\": 1, \"name\": \"一楼办公室\", \"create_timestamp\": 1760169378}]"},
  "result": true
}
````

⚠ `map_list` 是 JSON 字符串；没有地图时是 `success: true, map_list: "[]"`（旧文档写的 `success=false, "no map"` 不对）。
查询出错时 `success: false, map_list: "[]"`。

##### 2.2.14 获取当前地图

**Service**: `/get_current_map_id`　**Service Type**: `aid_robot_msgs/srv/GetCurrentMap`

````
{"op": "call_service", "service": "/get_current_map_id", "args": {}}
````

响应：

````
{"values": {"success": true, "map_id": 1, "map_file": "/home/unitree/maps/1790604029000", "map_name": "一楼办公室"}, "result": true}
````

没有当前地图时 `success: false, map_id: 0`。`map_file` 是绝对路径。

##### 2.2.15 获取地图图片 ⚠

**Service**: `/get_map_image`　**Service Type**: `aid_robot_msgs/srv/MapImage`

````
{"op": "call_service", "service": "/get_map_image", "args": {"id": 1}}
````

响应（旧文档误贴了 get_current_map_id 的响应）：

````
{
  "values": {
    "success": true,
    "map_file": "/home/unitree/maps/1790604029000/map.yaml",
    "map": {
      "info": {"resolution": 0.05, "width": 480, "height": 280,
               "origin": {"position": {"x": -2.0, "y": -7.0, "z": 0.0}, "orientation": {"x":0,"y":0,"z":0,"w":1}}},
      "data": [47, 57, 106, ...]
    }
  },
  "result": true
}
````

`map.data` 的编码与 2.2.4 相同（JPEG 的 base64 字符串的字节），解码方法见 2.2.4。

---

### 第三章 导航管理

导航调度由 `waypoint_manage_node` 负责：收到目标后调用 Nav2 的 `navigate_to_pose`，并发布 `/task_status`。
⚠ 前提：定位模式下（Nav2 随定位一起启动）。原前端在导航/巡逻页面会先调用 `/mode_set {"action": "patrol"}`。

#### 3.1 单点导航

##### 3.1.1 开始导航

**Topic**: `/nav_to_pose`　**Message Type**: `geometry_msgs/msg/PoseStamped`

````
{
  "op": "publish",
  "topic": "/nav_to_pose",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "pose": {
      "position": {"x": 2.0, "y": 3.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
````

- `orientation` 是到达后的朝向。
- ⚠ 如果已有任务在执行或暂停（`/task_status` 的 status 为 1 或 4），新目标会被**忽略**。要先
  `/patrol_control {"cmd": "cancel"}` 再发。

##### 3.1.2 `/move_base_simple/goal` ⚠（不支持）

G1 上没有节点订阅这个话题，发了不会有任何反应。请用 3.1.1。

##### 3.1.3 直接调用 Nav2 Action（不推荐）

**Action**: `/navigate_to_pose`　**Type**: `nav2_msgs/action/NavigateToPose`

可以直接发 Nav2 目标（格式同旧文档），但这样**绕过了 `waypoint_manage`**：`/task_status` 不会更新，
`/patrol_control` 也暂停/取消不了它。前端请用 3.1.1。

##### 3.1.4 暂停 / 继续 / 取消（单点和巡逻共用）⚠

**Service**: `/patrol_control`　**Service Type**: `aid_robot_msgs/srv/PatrolControl`

⚠ 服务定义里**只有 `cmd` 一个字段**（`pause` / `resume` / `cancel`）。旧文档里的 `patrol_count`、`patrol_duration`
后端没有实现，带上它们调用会直接报错：`Message type aid_robot_msgs/PatrolControl_Request does not have a field patrol_count`。

````
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "pause"}}
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "resume"}}
{"op": "call_service", "service": "/patrol_control", "args": {"cmd": "cancel"}}
````

响应：`{"success": true, "message": ""}`。没有任务在执行时调用也返回 `success: true`，但什么都不做。

#### 3.2 路线巡逻

##### 3.2.1 多点巡逻

**Topic**: `/patrol_path`　**Message Type**: `nav_msgs/msg/Path`

````
{
  "op": "publish",
  "topic": "/patrol_path",
  "msg": {
    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
    "poses": [
      {"header": {"frame_id": "map"}, "pose": {"position": {"x": 2.0, "y": 3.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}},
      {"header": {"frame_id": "map"}, "pose": {"position": {"x": 5.0, "y": 1.0, "z": 0.0}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}
    ]
  }
}
````

- 按顺序依次经过，走完最后一个点回到第一个点，**一直循环**，直到 `/patrol_control cancel`。
- 每个点的朝向会被改成"朝向下一个点"，传入的 orientation 不起作用。
- 某个点到不了（Nav2 失败）会跳过，继续下一个。
- 暂停 / 继续 / 取消：见 3.1.4。圈数和时长限制目前没有实现。

#### 3.3 任务状态

**Topic**: `/task_status`　**Message Type**: `aid_robot_msgs/msg/AidTaskStatus`（`waypoint_manage` 每秒发布一次）

````
{"op": "subscribe", "topic": "/task_status", "type": "aid_robot_msgs/msg/AidTaskStatus"}
````

接收：`{"msg": {"status": 1, "task_type": 0}}`

| status | 含义 | task_type | 含义 |
|---|---|---|---|
| 0 | 空闲 | 0 | 单点导航 |
| 1 | 执行中 | 1 | 巡逻 |
| 2 | 成功 | | |
| 3 | 失败 | | |
| 4 | 暂停 | | |
| 5 | 取消 | | |

---

### 第四章 模式设置

**Service**: `/mode_set`　**Service Type**: `aid_robot_msgs/srv/StatusChange`（`robot_status_manager`）

````
{"op": "call_service", "service": "/mode_set", "args": {"action": "localization"}}
````

响应：`{"values": {"message": "ok"}, "result": true}`，失败为 `"err"`。

| action | 作用 |
|---|---|
| `mapping` | 停止定位/导航，启动 Lightning 建图 |
| `localization` | 按数据库"当前地图"启动定位 + Nav2 |
| `patrol` | 控制模式切到导航/巡逻；⚠ 必须已经在定位模式，否则返回 `err` |
| `remote_control` | 控制模式切到遥控 |
| `idle` | 停止建图（不保存）/ 回到空闲 |

⚠ `robot.launch.py` 用 `start_backend:=false` 启动时，进程由顶层 launch 管理，`/mode_set` 不能切换建图/定位，
只能切换 `patrol` / `remote_control`。切换结果可以订阅 1.4 `/robot_status` 确认。

---

### 第五章 语义地图（新增）

说明：说一句话（例如"我想喝水"），在**当前地图的位置点**里找到意思最接近的一个（例如"茶水间"），
可以直接让机器人过去。由新增的 `semantic_map_ros/semantic_map_server` 提供：

````
docker compose -f src/semantic_map_ros/docker/docker-compose.yml up -d    # 启动向量数据库 Qdrant（只需一次）
ros2 launch semantic_map_ros semantic_map.launch.py                       # Thor 上加 use_fp16:=true
````

工作方式：
1. 位置点就是 2.1 的 `waypoint_node`，**给点位写上 `description`**（写成人会怎么说，例如"口渴""没电了"），匹配更准。
2. 每个点的"名称 + 描述"由 BGE-M3 模型转成向量，存到 Qdrant（每张地图一个集合 `semantic_map_<地图id>`）；**坐标不参与匹配**。
3. 搜索时把这句话也转成向量，找最相似的点；最高分低于阈值（默认 0.52）就算"没找到"，机器人不动。
4. 点位有增删改后，**下一次搜索会自动重建**，一般不需要手动重建。

##### 5.1 搜索

**Service**: `/semantic_map/search`　**Service Type**: `aid_robot_msgs/srv/SetString`

````
{"op": "call_service", "service": "/semantic_map/search", "args": {"data": "我想喝水"}}
````

响应：`success` 表示服务是否执行成功；`message` 是 JSON 字符串：

````
{
  "values": {
    "success": true,
    "message": "{\"query\": \"我想喝水\", \"found\": true, \"map_id\": 1, \"threshold\": 0.52, \"rebuilt\": false,
                \"best\": {\"id\": 1, \"name\": \"茶水间\", \"description\": \"有饮水机和咖啡机，可以接水、喝水、泡茶\",
                          \"frame_id\": \"map\", \"x\": 12.5, \"y\": 4.0, \"z\": 0.0, \"yaw\": 1.5708,
                          \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.7071, \"w\": 0.7071}, \"score\": 0.6767},
                \"candidates\": [ ...前 3 名，格式同 best... ],
                \"source\": \"search\", \"issues\": []}"
  },
  "result": true
}
````

| 字段 | 含义 |
|---|---|
| `found` | 最高分 ≥ 阈值时为 true |
| `best` | 找到时是第一名，否则为 null |
| `candidates` | 前 3 名（无论是否达到阈值，方便调试） |
| `score` | 余弦相似度，越接近 1 越像 |
| `rebuilt` | 这次搜索前是否因为点位变化自动重建了语义库 |
| `source` | 请求来源（见 5.5），调试用 |
| `issues` | 收到的原文里发现的问题，例如 `首尾有空白：换行`、`含不可见字符：零宽空格`；没问题时为 `[]` |

`data` 可以是一句普通文本，也可以是 JSON 字符串 `{"text": "我想喝水", "source": "asr"}`（见 5.5）。
送进模型前只去掉不可见字符和首尾空白，不改错字、不删标点。

失败时 `success: false`，`message` 是原因，例如 `no current map (select a map first)`、
`service /get_map_point_list not available`、`empty query`。

##### 5.2 一句话导航

**Service**: `/semantic_map/go`　**Service Type**: `aid_robot_msgs/srv/SetString`

````
{"op": "call_service", "service": "/semantic_map/go", "args": {"data": "我要上厕所"}}
````

- 先做 5.1 的搜索；找到时如果已有任务在执行/暂停，先 `/patrol_control cancel`，再把该点发到 `/nav_to_pose`
  （和 3.1.1 完全相同，由 `waypoint_manage` 执行，进度看 `/task_status`）。
- 响应的 `message` 同 5.1，多一个字段 `"navigating": true/false`（没找到时为 false，机器人不动）。
- 和普通导航一样需要在定位模式下；前端会先调 `/mode_set patrol`。

##### 5.3 手动重建语义库

**Service**: `/semantic_map/rebuild`　**Service Type**: `std_srvs/srv/Trigger`

````
{"op": "call_service", "service": "/semantic_map/rebuild", "args": {}}
````

响应：`{"success": true, "message": "{\"map_id\": 1, \"count\": 6}"}`（count 是写入的点位数）。

##### 5.4 参数与调阈值

`src/semantic_map_ros/config/semantic_map.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `qdrant_host` / `qdrant_port` | localhost / 6333 | Qdrant 地址 |
| `model_name` | BAAI/bge-m3 | 第一次运行自动下载约 2.3 GB，可填本地目录 |
| `use_fp16` | false | Thor（CUDA）上设 true |
| `score_threshold` | 0.52 | 低于它算"没找到" |
| `top_k` | 3 | 返回的候选数 |
| `text_in_topic` | /semantic_map/text_in | 话题输入（5.5），填 "" 关闭 |
| `text_in_action` | search | 话题输入收到后：search 只搜索；go 找到就导航（launch 参数 `text_in_action:=go`） |
| `log_file` | ~/maps/semantic_map_log.jsonl | 调试记录文件（5.6），填 "" 不写；超过 20 MB 滚动成 `.1` |
| `history_size` | 200 | `/semantic_map/history` 返回的最近条数 |

用真实点位调阈值：`python3 -m semantic_map_ros.evaluate --queries <句子.json>`（格式见
`config/eval_queries_example.json`），它会给出推荐阈值。

##### 5.5 其他模块（ASR 等）怎么把文本发过来

两种方式任选，效果相同。**建议带上 `source`**，调试记录里才分得清是谁发的。

**方式一：调服务**（能拿到应答，知道找没找到）——同 5.1 / 5.2，`data` 用 JSON 字符串带来源：

````
{"op": "call_service", "service": "/semantic_map/go",
 "args": {"data": "{\"text\": \"我想喝水\", \"source\": \"asr\"}"}}
````

ROS2 节点里直接调：`ros2 service call /semantic_map/search aid_robot_msgs/srv/SetString "{data: '我想喝水'}"`

**方式二：发话题**（只管发，不等结果；适合 ASR 识别完一句就往外丢）

**Topic**: `/semantic_map/text_in`　**Message Type**: `std_msgs/msg/String`

````
{"op": "publish", "topic": "/semantic_map/text_in", "msg": {"data": "我想喝水"}}
````

- `data` 同样可以是 `{"text": "...", "source": "asr"}`；不带 source 时来源记为 `topic`。
- 默认只搜索不导航（`text_in_action: search`），结果看 5.6 的调试记录；要让它找到就走，启动时加 `text_in_action:=go`。
- 走导航时和 5.2 一样，需要机器人已经在定位 + 导航模式（`/mode_set patrol`），这个由调用方或网页负责。
- 话题没有应答；出错（例如发来空内容、没有选地图）只会记在调试记录里。

##### 5.6 调试：看到别人发来的原文和匹配得分

每一次请求（网页、服务、话题，成功或失败）都会留下一条记录，用来判断问题出在哪一边：
**原文就不对**（错字、多了空格/换行/乱码）是发送方的问题；**原文没问题但得分低或匹配错**，是点位描述或阈值的问题。

看记录的三种方法：

1. **网页**："点位与语义"页 → 语义地图 → 调试记录。实时显示所有来源的请求，看不见的字符用黄底标出；
   可勾"只看有问题的"；上面的"模拟 ASR"输入框可以往 `/semantic_map/text_in` 发一句，检查话题链路。
2. **话题** `/semantic_map/debug`（`std_msgs/msg/String`，每条一个 JSON）：`ros2 topic echo /semantic_map/debug`
3. **文件** `~/maps/semantic_map_log.jsonl`（每行一条，重启不丢）：`tail -f ~/maps/semantic_map_log.jsonl`

另有服务 `/semantic_map/history`（`std_srvs/srv/Trigger`），`message` 是最近 200 条记录的 JSON 数组（网页打开时用它补齐）。

一条记录：

````
{"time": "2026-09-29T10:15:02.318", "entry": "topic", "source": "asr",
 "raw": " 我想\u200b喝水\n", "raw_visible": "␠我想⟨U+200B⟩喝水\\n", "length": 7,
 "issues": ["首尾有空白：换行、空格", "含不可见字符：零宽空格"], "text": "我想喝水", "go": false,
 "ok": true, "map_id": 1, "found": true, "best": "茶水间", "threshold": 0.52,
 "candidates": [{"id": 1, "name": "茶水间", "score": 0.6767}, {"id": 3, "name": "会议室", "score": 0.41},
                {"id": 2, "name": "前台", "score": 0.39}],
 "rebuilt": false, "navigating": false, "elapsed_ms": 86}
````

| 字段 | 含义 |
|---|---|
| `entry` | 从哪个入口进来：`search` / `go` 服务，或 `topic` 话题 |
| `source` | 发送方自己报的来源；没报时等于 `entry` |
| `raw` | 收到的原文，一个字符都不改 |
| `raw_visible` | 同上，但把看不见的字符写出来：`␠` 首尾空格、`\n` 换行、`⟨U+200B⟩` 零宽空格等 |
| `issues` | 原文的问题（首尾空白、不可见字符、中间换行、乱码替换符 U+FFFD、清理后为空） |
| `text` | 实际送进模型的文本 |
| `candidates` | 前 3 名及得分；`found=false` 时可以看到最高分差阈值多少 |
| `ok` / `error` | 是否处理成功；失败原因（例如 `empty query`、`no current map`） |
| `navigating` | 是否已发出导航目标 |
| `elapsed_ms` | 处理耗时（含模型推理） |

---

### 附录 A 修订记录（2026-09，对照源码 + 仿真实测）

| 章节 | 旧文档 | 实际情况 / 修改 |
|---|---|---|
| 0 | 无 | 新增连接方式；string 字段传 JSON 字符串；多余字段会报错 |
| 全文 | 多数服务 `args` 写成数组 `[{...}]` | 实测会被 rosbridge 拒绝，全部改成对象 `{...}` |
| 1.1 重定位 | `/start_init_pose` | 没有节点订阅；`/aid_init_pose → /initialpose` 也无人处理 → 标注"暂未实现" |
| 1.2 | — | 补充：只有定位/建图时才发布 |
| 1.3 电量 | 响应话题写成 `/battery_data`；示例 percentage 80.0 | 话题是 `/battery_state`；percentage 为 0~1；charge 等为 NaN；充电看 `power_supply_status`；建议 cbor 订阅 |
| 1.4 | 无 | 新增 `/robot_status` |
| 2.1.1 新增点位 | `data` 写成对象，JSON 缺逗号 | `data` 必须是 JSON 字符串；可选 `description` 字段 |
| 2.1.3 修改点位 | `data` 写成对象 | JSON 字符串，整体替换 |
| 2.1.4 点位列表 | `message` 写成数组、`point_list` 写成对象 | 两层都是 JSON 字符串 |
| 2.2.1 / 4 mode_set | 响应 `{"msg": "ok"}` | 响应字段是 `message` |
| 2.2.3 保存地图 | 只有 `/add_map`，类型写 MapOperation，map_file 写"绝对路径" | 先 `/aid_save_map` 再 `/add_map`；类型 MapOperationAdd；路径相对家目录 |
| 2.2.4 建图预览 | 由 map_manager_server 输出 | 实际由 `aid_robot_py/map_transform_node` 发布；补充 data 的编码方式 |
| 2.2.5 切换地图 | — | 补充：还需 `/mode_set localization` 才会加载 |
| 2.2.9 禁行线 | `/draw_no_go_lines` | 不存在；实际是 `/set_forbidden` + `/aid_draw_forbidden_line`（+ `/get_forbidden`） |
| 2.2.10 橡皮擦 | — | 补充 grayscale 含义、会改写文件、需重新加载 |
| 2.2.12 更新地图 | "可以输入一个 newmap" | `data` 是 `{"name": ...}` 的 JSON 字符串 |
| 2.2.13 地图列表 | 没有地图时 success=false | 实际 success=true、`"[]"` |
| 2.2.15 地图图片 | 响应贴成了 get_current_map_id 的 | 改为 `{success, map, map_file}`，说明图片编码 |
| 3.1.1 | — | 补充：有任务时新目标被忽略，需先 cancel |
| 3.1.2 | `/move_base_simple/goal` | 没有订阅者，删除 |
| 3.1.3 | Nav2 action | 注明会绕过 task_status 和 patrol_control |
| 3.1.x / 3.2.4 | `patrol_count`、`patrol_duration` | PatrolControl.srv 只有 `cmd`，带这两个字段调用会报错（仿真实测） |
| 3.2.1 | — | 补充：无限循环、朝向自动计算、失败点跳过 |
| 5 | 无 | 新增语义地图接口 |
| 5.5 / 5.6 | 无 | 新增话题输入 `/semantic_map/text_in`、请求来源 `source`、调试记录（`/semantic_map/debug`、`/semantic_map/history`、日志文件） |
