##          海尔S01通信接口文档

### 第一章 状态管理

#### 1.1重定位 (暂无)

说明：将机器人的定位还原零点，在整体场景中通常为充电电前的位置，在建图时的起始点

**Topic**: `/start_init_pose`  

**Message Type**:`geometry_msgs/msg/PoseStamped`

`````
{
  "op": "publish",
  "topic": "/start_init_pose",
  "msg": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  },
  "id": "<optional_string>"
}
`````

#### 1.2机器人当前位置 

说明：机器人的实时位置，用于在前端显示机器人当前在地图中的位置

**Topic**: `/base_link_pose` 
**Message Type**: `geometry_msgs/msg/PoseStamped`

`````
{
  "op": "subscribe",
  "topic": "/base_link_pose",
  "type": "geometry_msgs/msg/PoseStamped",
  "compression": "cbor",
  "id": "<optional_string>"
}
`````

**接收的消息**:

````
{
  "op": "publish",
  "topic": "/base_link_pose",
  "msg": {
    "header": {
      "stamp": {
        "sec": 1234567890,
        "nanosec": 123456789
      },
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 1.234,
        "y": 5.678,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  },
  "id": "<optional_string>"
}
````

#### 1.3获取电量信息 

**Topic**: `/battery_state` 
**Message Type**: `sensor_msgs/msg/BatteryState`

请求：

````
{
  "op": "subscribe",
  "topic": "/battery_state",
  "type": "sensor_msgs/msg/BatteryState",
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "publish",
  "topic": "/battery_data",
  "msg": {
    "header": {
      "stamp": {
        "sec": 1234567890,
        "nanosec": 123456789
      },
      "frame_id": ""
    },
    "voltage": 12.6,
    "temperature": 25.0,
    "current": 1.5,
    "charge": 4.0,
    "capacity": 4.2,
    "design_capacity": 5.0,
    "percentage": 80.0,
    "power_supply_status": 2,
    "power_supply_health": 1,
    "power_supply_technology": 2,
    "present": true,
    "cell_voltage": [3.7, 3.7, 3.7, 3.7],
    "cell_temperature": [25.0, 25.0, 25.0, 25.0],
    "location": "base",
    "serial_number": "BAT123456789"
  },
  "id": "<optional_string>"
}
`````



### 第二章 地图管理

#### 2.1 地图位置点管理

##### 2.1.1  地图新增位置点

**Service**: `/add_point` 
**Service Type**: `aid_robot_msgs/srv/OperationAdd`

````
- 入参：
  - uint32 map_id 对应地图的id 
  - string frame_id 对应地图的frame_id，默认填map 
  - string data 位置点内容 json字符串
  - string data_type 固定值："waypoint_node"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败） 
  - string message 消息，分以下几种情况 1. success=true,message="ok" 新增成功 2. success=false,message="error" 新增失败，数据库操作过程中出现问题
````

请求：

````
{
  "op": "call_service",
  "service": "add_point",
  "args": {
    "map_id": 1,
  "frame_id":"map"
    "data": {
    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "name":"test1"
  },
    "data_type": "waypoint_node"
  },
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "add_point",
  "values": {
    "success": true,
    "message": "ok"
  },
  "id": "<optional_string>"
}

`````


##### 2.1.2  删除指定位置点

**Service**: `/delete_point` 
**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
- 入参：
  - uint32 id 位置点记录的id
  - string data_type 固定值："waypoint_node"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败） 
  - string message 消息，分以下几种情况 1. success=true,message="ok" 删除成功 2. success=false,message="error" 删除失败，数据库操作过程中出现问题
````

请求：

````
{
  "op": "call_service",
  "service": "delete_point",
  "args": {
    "id": 1,
    "data_type": "waypoint_node"
  },
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "delete_point",
  "values": {
    "success": true,
    "message": "ok"
  },
  "id": "<optional_string>"
}

`````


##### 2.1.3  修改位置点参数

**Service**: `/update_point` 
**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

````
- 入参：
  - uint32 id 位置点记录的id 
  - string data 更新的内容（json字符串）  
  - string data_type 固定值："waypoint_node"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败） 
  - string message 消息，分以下几种情况 1. success=true,message="ok" 更新成功 2. success=false,message="error"或message="data not found" 更新失败，数据库操作过程中出现问题
````

请求：

````
{
  "op": "call_service",
  "service": "update_point",
  "args": {
    "id": 1,
    "data": {
    "position": {"x": 1.0, "y": 2.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "name":"test1"
  },
    "data_type": "waypoint_node"
  },
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "update_point",
  "values": {
    "success": true,
    "message": "ok"
  },
  "id": "<optional_string>"
}

`````

##### 2.1.4 获取地图对应的位置点记录列表

**Service**: `/get_map_point_list` 
**Service Type**: `aid_robot_msgs/srv/MapLinkedDataList`

````
- 入参：
  - uint32 map_id 对应地图的id
  - string data_type 固定值："waypoint_node"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败） 
  - string message 站点序列的json报文
````

请求：

````
{
  "op": "call_service",
  "service": "get_map_point_list",
  "args": {
    "map_id": 1,
    "data_type": "waypoint_node"
  },
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "get_map_point_list",
  "values": {
    "success": true,
    "message": [
    { 
      "id": 1,
      "frame_id": "map",
      "point_list":{
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "name":"test1"
      }
    },
    { 
      "id": 2,
      "frame_id": "map",
      "point_list":{
        "position": {"x": 1.0, "y": 2.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "name":"test2"
      }
    }
  ]
  },
  "id": "<optional_string>"
}
`````

#### 2.2地图管理

##### 2.2.1 开始建图 

说明：切换到mapping模式启动建图

请求：

`````
{
  "op": "call_service",
  "service": "/mode_set",
  "args": [
    {
      "action": "mapping" 
    }
  ],
  "id": "<optional_string>"
}
`````

响应：

````
{
  "op": "service_response",
  "service": "/mode_set",
  "result": true,
  "values": {
    "msg": "ok"
  },
  "id": "<optional_string>"
}
````

##### 2.2.2  取消建图 

说明：直接切换到idle模式，不做地图保存

请求：

`````
{
  "op": "call_service",
  "service": "/mode_set",
  "args": {
    "action": "idle"
  },
  "id": "robot_ws_menu_7"
}
`````

响应：

````
{
  "op": "service_response",
  "service": "/mode_set",
  "values": {
    "message": "ok"
  },
  "result": true,
  "id": "robot_ws_menu_7"
}
````

##### 2.2.3 保存到数据库 

**Service**: `/add_map` 
**Service Type**: `aid_robot_msgs/srv/MapOperation`

说明：保存地图记录到数据库 ：调用 /add_map ，该服务把地图的逻辑信息（名称、文件路径等）写入 SQLite 的 map 表，为后续切换/管理提供数据支持

`````
- 入参：
  - string map_name 用户设置的地图名字
  - string map_file 建图后，地图yaml文件的绝对路径
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - string message 消息，分以下几种情况
    1. success=true,message="ok" 新增成功
    2. success=false,message="error" 新增失败，数据库操作过程中出现问题
`````

请求：

````
{
  "op": "call_service",
  "service": "/add_map",
  "args": [
    {
      "map_name": "<string>name",
      "map_file": "<string>file_path"
    }
  ],
  "id": "<optional_string>"
}
````

响应：

````
{
  "op": "service_response",
  "service": "/add_map",
  "result": true,
  "values": {
    "success": true,
    "message": "ok"
  },
  "id": "<optional_string>"
}
````

##### 2.2.4 建图订阅 

说明：用于建图过程中的实时预览

**Topic**: `/map_base64`  
**Message Type**: `nav_msgs/msg/OccupancyGrid`

说明: 订阅建图数据 ：脚本直接订阅 /map_base64 Topic（nav_msgs/msg/OccupancyGrid，PNG 压缩），。该 Topic 由 map_manager_server.py:388 输出，将地图图像编码为 base64，用于前端实时预览建图过程

```json
{
  "op": "subscribe",
  "topic": "/map_base64",
  "type": "nav_msgs/msg/OccupancyGrid",
  "compression": "png",
  "id": "<optional_string>"
}
```

**接收的消息**:

```json
{
  "op": "publish",
  "topic": "/map_base64",
  "msg": {
    "info": {
      "resolution": 0.05,
      "width": 1024,
      "height": 768,
      "origin": {
        "position": {
          "x": -25.6,
          "y": -19.2,
          "z": 0.0
        },
        "orientation": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0,
          "w": 1.0
        }
      }
    },
    "data": "base64_encoded_image_data"
  },
  "id": "<optional_string>"
}
```

##### 2.2.5 切换地图 

**Service**: `set_current_map_id` 
**Service Type**: `aid_robot_msgs/srv/SetCurrentMap`

````
- 入参：
  - uint32 id 地图的id
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
````

请求：

````
{
  "op": "call_service",
  "service": "/set_current_map_id",
  "args": {
    "id": "<uint32>" map_id
  },
  "id": "<optional_string>"
}

````

响应：

`````
{
  "op": "service_response",
  "service": "set_current_map_id",
  "result": true,
  "values": {
    "success": true
    "result": true
  },
  "id": "<optional_string>"
}
`````

##### 2.2.9 设置禁行线 

**Service**: `/draw_no_go_lines` 
**Service Type**: `aid_robot_msgs/srv/DrawPicture`

````
{
  "op": "call_service",
  "service": "/draw_no_go_lines",
  "args": [
    {
      "frame_id": "map",
      "type": "line",
      "data": [
        {
          "start": {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0
          },
          "end": {
            "x": 3.0,
            "y": 4.0,
            "z": 0.0
          }
        }
      ]
    }
  ],
  "id": "<optional_string>"
}
````

**响应**:

````
{
  "op": "service_response",
  "service": "/draw_no_go_lines",
  "result": true,
  "values": {
    "success": true,
    "message": "No-go lines drawn successfully"
  },
  "id": "<optional_string>"
}
````

##### 2.2.10 橡皮擦功能 

**Service**: `/map_editor` 
**Service Type**: `aid_robot_msgs/srv/DrawPicture`

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
   {
    "center_point": {
     "x": 0.0,
     "y": 0.0,
     "z": 0.0
    },
    "side_length": 0.5,
    "grayscale": 0
   }
  ]
 },
 "id": ""
}
````

**响应**:

````
{
 "op": "service_response",
 "service": "/map_editor",
 "values": {
  "success": true,
  "message": "Map updated successfully"
 },
 "result": true,
 "id": ""
}
````

##### 2.2.11 删除地图 

**Service**: `delete_map` 
**Service Type**: `aid_robot_msgs/srv/OperationDelete`

````
- 入参：
  - uint32 id 需要删除的地图的id
  - string data_type 固定值："map"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - string message 返回消息，默认'OK'
````

请求：

````
{
  "op": "call_service",
  "service": "/delete_map",
  "args": [
    {
      "id": "<uint32>map_id",
      "data_type": "map"
    }
  ],
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "/delete_map",
  "result": true,
  "values": {
    "success": true,
    "message": "OK"
  },
  "id": "<optional_string>"
}
`````



##### 2.2.12 更新地图 

**Service**: `update_map ` 
**Service Type**: `aid_robot_msgs/srv/OperationUpdate`

`````
- 入参：
  - uint32 id 要更新的地图的id
  - string data 更新的内容（json字符串）
  - string data_type 固定值："map"
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - string message 返回消息，分以下几种情况
    1. success=false,message="data not found" 数据库中查不到对应的数据
    2. success=true,message="ok" 数据库中查到对应的数据并正常更新了
    3. success=true,message="error" 数据库中查到对应的数据，但没有正常更新成功
`````

请求：

```
{
  "op": "call_service",
  "service": "/update_map",
  "args": [
    {
      "id": "<uint32>map_id",
      "data": "<json_string>", 可以输入一个newmap，其余字段不建议
      "data_type": "map"
    }
  ],
  "id": "<optional_string>"
}
```

响应：

````
{
  "op": "service_response",
  "service": "/update_map",
  "values": {
    "success": true,
    "message": "ok"
  },
  "result": true,
  "id": "<optional_string>"
}
````

##### 2.2.13 获取地图列表 

**Service**: `get_map_list` 
**Service Type**: `aid_robot_msgs/srv/MapList`

`````
- 入参：
  - 无
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - string map_list 地图列表，分以下几种情况
    1. success=false,map_list="no map" 数据库中查不到对应的数据
    2. success=false,map_list="error" 查询出现问题
    3. success=true,map_list="[{name, id}, {name, id}, ...]" 查到地图数据，转换为json字符串

`````

请求：

````
{
  "op": "call_service",
  "service": "/get_map_list",
  "args": [],
  "id": "<optional_string>"
}
````

响应：

````
{
"op": "service_response",
  "service": "/get_map_list",
  "values": {
    "success": true,
    "map_list": "[{\"id\": 1, \"name\": \"lmw\", \"create_timestamp\": 1760169378}]"
  },
  "result": true,
  "id": "<optional_string>"
}
````

##### 2.2.14 获取当前使用中的地图ID 

**Service**: `get_current_map_id` 
**Service Type**: `aid_robot_msgs/srv/GetCurrentMap`

````
- 入参：
  - 无
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - uint32 map_id 地图id，获取成功为实际id，获取失败或者当前没有使用中的地图，返回值为0
````

请求：

````
{
  "op": "call_service",
  "service": "/get_current_map_id",
  "args": [],
  "id": "<optional_string>"
}
````

响应：

````
{
"op": "service_response",
"service": "/get_current_map_id", 
"values": {
	"success": true,
    "map_id": map_id,
    "map_file": "<string>", path_to_map
    "map_name": "<string>" map_name
    }, 
"result": true
}
````

##### 2.2.15 获取当前使用中的地图图像

**Service**: `get_map_image` 
**Service Type**: `aid_robot_msgs/srv/MapImage`

````
- 入参：
  - 无
- 返回值：
  - bool success 成功失败的标志（true成功，false失败）
  - uint32 map_id 地图id，获取成功为实际id，获取失败或者当前没有使用中的地图，返回值为0
````

请求：

````
{
  "op": "call_service",
  "service": "/get_map_image",
  "args": [
    {
      "id": "<uint32>"
    }
  ],
  "id": "<optional_string>",
  "compression": "cbor-raw"
}
````

响应：

````
{
"op": "service_response",
"service": "/get_current_map_id", 
"values": {
	"success": true,
    "map_id": map_id,
    "map_file": "<string>", path_to_map
    "map_name": "<string>" map_name
    }, 
"result": true
}
````

### 

### 第三章 导航管理

#### 3.1 单点导航

#####   3.1.1 开始导航

**Topic**: `/nav_to_pose` 
**Message Type**: `geometry_msgs/msg/PoseStamped`

`````
{
  "op": "publish",
  "topic": "/nav_to_pose",
  "msg": {
    "header": {
      "stamp": {"sec": 1234567890, "nanosec": 123456789},
      "frame_id": "map"
    },
    "pose": {
      "position": {"x": 2.0, "y": 3.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "id": "<optional_string>"
}
`````

#####   3.1.2 Topic 单点导航 (Simple Goal Topic Navigation)

> 说明：以下接口使用 `/move_base_simple/goal` 单点目标话题。若机器人侧仍保留项目现有的 `/nav_to_pose` 自定义话题，可按机器人端实际能力二选一接入。

**Topic**: `/move_base_simple/goal`
**Message Type**: `geometry_msgs/msg/PoseStamped`

```json
{
  "op": "publish",
  "topic": "/move_base_simple/goal",
  "msg": {
    "header": {
      "stamp": {
        "sec": 0,
        "nanosec": 0
      },
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 1.25,
        "y": 2.50,
        "z": 0.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.7071068,
        "w": 0.7071068
      }
    }
  },
  "id": "<optional_string>"
}
```

#####   3.1.3 Nav2 Action 单点导航 (Nav2 Action Goal Navigation)

**Action**: `/navigate_to_pose`
**Action Type**: `nav2_msgs/action/NavigateToPose`

**发送目标**:

```json
{
  "op": "send_action_goal",
  "action": "/navigate_to_pose",
  "args": [
    {
      "pose": {
        "header": {
          "stamp": {
            "sec": 0,
            "nanosec": 0
          },
          "frame_id": "map"
        },
        "pose": {
          "position": {
            "x": 1.25,
            "y": 2.50,
            "z": 0.0
          },
          "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.7071068,
            "w": 0.7071068
          }
        }
      },
      "behavior_tree": ""
    }
  ],
  "id": "navigate_to_pose_goal_001"
}
```

**动作反馈**:

```json
{
  "op": "action_feedback",
  "action": "/navigate_to_pose",
  "values": [
    {
      "current_pose": {
        "header": {
          "stamp": {
            "sec": 1234567890,
            "nanosec": 123456789
          },
          "frame_id": "map"
        },
        "pose": {
          "position": {
            "x": 1.00,
            "y": 2.00,
            "z": 0.0
          },
          "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.3826834,
            "w": 0.9238795
          }
        }
      },
      "navigation_time": {
        "sec": 12,
        "nanosec": 0
      },
      "estimated_time_remaining": {
        "sec": 8,
        "nanosec": 500000000
      },
      "number_of_recoveries": 0,
      "distance_remaining": 1.42
    }
  ],
  "id": "navigate_to_pose_goal_001"
}
```

**动作结果**:

```json
{
  "op": "action_result",
  "action": "/navigate_to_pose",
  "result": true,
  "values": [
    {
      "error_code": 0,
      "error_msg": ""
    }
  ],
  "id": "navigate_to_pose_goal_001"
}
```

**取消目标**:

```json
{
  "op": "cancel_action_goal",
  "action": "/navigate_to_pose",
  "id": "navigate_to_pose_goal_001"
}
```


#####   3.1.2 暂停导航

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

说明：`/patrol_control` 为导航和巡逻共用的控制服务。单点导航恢复时，`patrol_count` 和 `patrol_duration` 不参与单点导航的执行逻辑，但请求中仍应按服务定义传入 `-1`。

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "pause",
      "patrol_count": -1,
      "patrol_duration": -1.0
    }
  ],
  "id": "<optional_string>"
}
`````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````

#####   3.1.3 恢复导航

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

说明：`/patrol_control` 为导航和巡逻共用的控制服务。单点导航恢复时，`patrol_count` 和 `patrol_duration` 不参与单点导航执行，但应按服务定义传入 `-1`。

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "resume",
      "patrol_count": -1,
      "patrol_duration": -1.0
    }
  ],
  "id": "<optional_string>"
}
`````

无限制巡逻示例：

````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "resume",
      "patrol_count": -1,
      "patrol_duration": -1.0
    }
  ],
  "id": "<optional_string>"
}
````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````


##### 3.1.4 取消导航

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "cancel" 
    }
  ],
  "id": "<optional_string>"
}
`````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````

#### 3.2  路线巡逻

##### 3.2.1 多点巡逻

**Topic**: `/patrol_path` 
**Message Type**: `nav_msgs/msg/Path`

说明：发布路径后，机器人开始依次执行路径中的点位。若需要设置巡逻圈数或总时长，应在发布路径后调用 [3.2.4 恢复巡逻](#324-恢复巡逻) 并传入相应参数；参数仅在 `cmd` 为 `resume` 时生效。

`````
{
  "op": "publish",
  "topic": "/patrol_path",
  "msg": {
    "poses": [
      {
        "header": {
          "stamp": {
            "sec": 1234567890,
            "nanosec": 123456789
          },
          "frame_id": "map"
        },
        "pose": {
          "position": {
            "x": 2.0,
            "y": 3.0,
            "z": 0.0
          },
          "orientation": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0
          }
        }
      }
    ]
  },
  "id": "<optional_string>"
}
`````

##### 3.2.2 巡逻暂停控制

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "pause" 
    }
  ],
  "id": "<optional_string>"
}
`````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````

##### 3.2.3取消巡逻

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "cancel" 
    }
  ],
  "id": "<optional_string>"
}
`````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````

#####   3.2.4 恢复巡逻

**Service**: `/patrol_control`  
**Service Type**: `aid_robot_msgs/srv/PatrolControl`

说明：用于恢复已暂停的巡逻，也用于在发布 `/patrol_path` 后配置巡逻限制。服务定义如下：

| 请求字段          | 类型    | 说明                                                         |
| ----------------- | ------- | ------------------------------------------------------------ |
| `cmd`             | string  | 固定为 `"resume"`，恢复或启动巡逻执行。                      |
| `patrol_count`    | int32   | 巡逻圈数限制。`-1` 表示不限制；大于 `0` 时，机器人完成指定圈数后结束任务。`0` 不建议使用，当前实现会在首圈完成后的限制检查中结束任务。 |
| `patrol_duration` | float64 | 巡逻总时长限制，单位为秒。`-1.0` 表示不限制；大于等于 `0` 时，到达设定时长后结束任务。 |

当圈数和时长同时设置时，任一限制先达到，巡逻即结束。当前机器人端在每完成一圈时检查限制条件，因此实际停止时间可能略晚于设定时长；`patrol_duration` 为 `0` 时同样会在首圈完成后停止。建议在每次启动或恢复巡逻时都完整传入这两个字段。

`````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "resume",
      "patrol_count": 3,
      "patrol_duration": 1800.0
    }
  ],
  "id": "<optional_string>"
}
`````

**响应**:

`````
{
  "op": "service_response",
  "service": "/patrol_control",
  "result": true,
  "values": {
    "success": true,
    "message": ""
  },
  "id": "<optional_string>"
}
`````

#### 3.3  任务状态发布

说明：机器人的执行单点和巡视的任务状态

**Topic**: `/task_status` 
**Message Type**: `aid_robot_msgs/msg/AidTaskStatus`

`````
{
  "op": "subscribe",
  "topic": "/task_status",
  "type": "aid_robot_msgs/msg/AidTaskStatus",
  "compression": "cbor",
  "id": "<optional_string>"
}
`````

**接收的消息**:

````
{
  "op": "publish",
  "topic": "/task_status",
  "msg": {
    "status": 1,
    "task_type": 0
  },
  "id": "<optional_string>"
}
````

无限制巡逻示例：

````
{
  "op": "call_service",
  "service": "/patrol_control",
  "args": [
    {
      "cmd": "resume",
      "patrol_count": -1,
      "patrol_duration": -1.0
    }
  ],
  "id": "<optional_string>"
}
````

status和task_type状态说明:
字段      类型      描述
status  int32 任务状态：0 - idle（空闲）
             1 - working（执行中）
             2 - success（成功完成）
             3 - failed（失败）
             4 - suspend（暂停）
             5 - cancel（取消）
             
task_type int32 任务类型： 0 - 单点导航
             1 - 巡视导航

### 第四章 模式设置 

说明：设置机器人的模式，对应机器人的不同工作状态支持一键切换 /mode_set 到 mapping/localization/patrol/remote_control/idle

**Topic**: `/mode_set` 
**Message Type**: `aid_robot_msgs/srv/StatusChange`

请求：

````
{
  "op": "call_service",
  "service": "/mode_set",
  "args": [
    {
      "action": "mapping" 
    }
  ],
  "id": "<optional_string>"
}
````

响应：

`````
{
  "op": "service_response",
  "service": "/mode_set",
  "result": true,
  "values": {
    "msg": "ok"
  },
  "id": "<optional_string>"
}
`````

