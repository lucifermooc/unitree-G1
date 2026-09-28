##          海尔S01通信接口文档

### 第一章 状态管理

#### 1.1重定位 

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

#### 1.3遥控控制 

**Topic**: `/cmd_vel_remote_ctrl` 
**Message Type**: `geometry_msgs/msg/Twist`

````
{
  "op": "publish",
  "topic": "/cmd_vel_remote_ctrl",
  "msg": {
    "linear": {
      "x": 0.5,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.3
    }
  },
  "id": "<optional_string>"
}
````

#### 1.4获取电量信息 

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

#### 1.5获取IP地址 

**Topic**: `/get_ip_addresses` 
**Message Type**: `aid_robot_msgs/srv/GetString`

请求：

````
{
  "op": "call_service",
  "service": "/get_ip_addresses",
  "args": [],
  "id": "<optional_string>"
}
````

响应：

````
{
  "op": "service_response",
  "service": "/get_ip_addresses",
  "values": {
    "success": true,
    "message": "IP addresses retrieved successfully",
    "result": "192.168.1.4"
  },
  "id": "<optional_string>"
}
````

#### 1.6头部电机控制 

`````
- 电机控制topic：/target_head_position (std_msgs/msg/Float32) 发布目标位置 (单位：弧度)
- 电机状态订阅topic：/joint_states (sensor_msgs/msg/JointState) 接收电机当前位置 速度 力矩
- 订阅限位topic：/head_upper_limit (std_msgs/msg/Float32) 接收头部电机上限位 (单位：弧度) /head_lower_limit (std_msgs/msg/Float32) 接收头部电机下限位
`````

**Topic**: `/target_head_position` 
**Message Type**: `std_msgs/msg/Float32`

请求：

``````
{
  "op": "publish",
  "topic": "/target_head_position",
  "msg": { "data": float32 }  取值范围3.29~2.94
}
``````

上限位请求：

````
{
  "op": "subscribe",
  "topic": "/head_upper_limit",
  "type": "std_msgs/msg/Float32",
  "id": "<optional_string>"
}
````

下限位请求：

`````
{
  "op": "subscribe",
  "topic": "/head_lower_limit",
  "type": "std_msgs/msg/Float32",
  "id": "<optional_string>"
}
`````

#### 1.7胸部LED控制 

说明： 通过GPIO199控制LED 

`````
打开GPIO199
echo 1 > /sys/devices/platform/soc/soc:gp5_gpios/RESERVED_QCM8550_GPIO199
发送低电平关闭LED灯
echo 1 0 > /sys/devices/platform/soc/soc:gp5_gpios/RESERVED_QCM8550_GPIO199
发送高电平开启LED灯
echo 1 1 > /sys/devices/platform/soc/soc:gp5_gpios/RESERVED_QCM8550_GPIO199
关闭GPIO199
echo 0 > /sys/devices/platform/soc/soc:gp5_gpios/RESERVED_QCM8550_GPIO199
`````

#### 1.8后盖按键版控制 

**Topic**: `/target_head_position` 
**Message Type**: `can_msgs/msg/Frame`

​    说明： id=0x105 的帧表示音量控制，data[0] 表示按键类型。 id=0x301 的帧表示关机接口，data字段全部为0

音量控制：

`````
{
  "op": "publish",
  "topic": "/CAN/can1/transmit",
  "msg": {
    "id": 261, // 0x105
    "data": [1, 0, 0, 0, 0, 0, 0, 0]
  },
  "id": "<optional_string>"
}
optional_string：and -s "input keyboard keyevent 24/25"
data[0] 类型说明:
  1 - 音量减小键按下 (kVolumeDownPress)
  2 - 音量减小键松开 (kVolumeDownRelease)
  3 - 音量增加键按下 (kVolumeUpPress)
  4 - 音量增加键松开 (kVolumeUpRelease)
  在执行音量减少（增加）按下的时候需要在执行完成之后额外执行一个安卓的指令，如下：
and -s "input keyboard keyevent 24" ---音量增加执行
and -s "input keyboard keyevent 25" ---音量减少执行
`````

软关机控制：

`````
{
  "op": "publish",
  "topic": "/CAN/can1/transmit",
  "msg": {
    "id": 769, // 0x301
    "data": [0, 0, 0, 0, 0, 0, 0, 0]
  },
  "id": "<optional_string>"
}
`````

#### 1.9 急停状态 

说明：机器人急停按钮状态。`data=1` 表示急停触发，`data=0` 表示急停释放。话题为 latched，新订阅者会立即收到最近一次状态。

**Topic**: `/e_stop` 
**Message Type**: `std_msgs/msg/Int32`

请求（订阅）：

````
{
  "op": "subscribe",
  "topic": "/e_stop",
  "type": "std_msgs/msg/Int32",
  "id": "<optional_string>"
}
````

**接收的消息**:

````
{
  "op": "publish",
  "topic": "/e_stop",
  "msg": {
    "data": 1
  },
  "id": "<optional_string>"
}
````

`data` 取值说明：

| 值   | 含义             |
| ---- | ---------------- |
| 0    | 急停释放（正常） |
| 1    | 急停触发（按下） |



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

说明：切换到mapping模式启动cartographer的建图

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

说明：保存地图记录到数据库 ：调用 /add_map 或 /map_management/add_map，该服务把地图的逻辑信息（名称、文件路径等）写入 SQLite 的 map 表，为后续切换/管理提供数据支持

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
  "service": "/map_management/add_map",
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
  "service": "/map_management/add_map",
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

##### 2.2.5 完成建图轨迹 

**Service**: `/finish_trajectory`  
**Service Type**: `cartographer_ros_msgs/srv/FinishTrajectory`

说明: 完成建图轨迹 ：调用 cartographer_ros_msgs/srv/FinishTrajectory，通知 Cartographer 将当前轨迹 trajectory_id 结束，停止继续积累扫描数据；避免未完成的轨迹拖尾

````
{
  "op": "call_service",
  "service": "/finish_trajectory",
  "args": {
      "trajectory_id": trajectory_id<int> "0"
    },
  "id": "<optional_string>"
}

````

响应

`````
{
  "op": "service_response",
  "service": "/finish_trajectory",
  "values": {
    "status": {
      "code": 0,
      "message": "Finished trajectory 0."
    }
  },
  "result": true,
  "id": "<optional_string>"
}
`````

##### 2.2.6 保存地图状态 

**Service**: `/write_state`  
**Service Type**: `cartographer_ros_msgs/srv/WriteState`

说明: 保存地图状态：调用 cartographer_ros_msgs/srv/WriteState（，把 Cartographer 的内部 SLAM 状态写成 .pbstream 文件，以便后续重新加载或继续建图；

````
{
  "op": "call_service",
  "service": "/write_state",
  "args": [
    {
      "filename": "<string>" /home/aidlux/maps/your_name.pbstream 
    }
  ],
  "id": "<optional_string>"
}
````

响应

`````
{
  "op": "service_response",
  "service": "/write_state",
  "values": {
    "status": {
      "code": 0,
      "message": "State written to '/home/aidlux/maps/lmw.pbstream'."
    }
  },
  "result": true,
  "id": "<optional_string>"
}

`````

##### 2.2.7 集成保存地图 

**Service**: `/aid_save_map`  
**Service Type**: `aid_robot_msgs/srv/MapOperation`

说明: 集成保存地图：触发 aid_robot_msgs/srv/MapOperation 的 /aid_save_map，该服务封装了“停止建图 → 保存 pbstream → 调用 map_saver 生成占据栅格”的整套流程

`````
{
  "op": "call_service",
  "service": "/aid_save_map",
  "args": [
    {
      "map_file_name": "<string>"  /maps/your_name
    }
  ],
  "id": "<optional_string>"
}
`````

 响应

````{
  "op": "service_response",
  "service": "/aid_save_map",
  "values": {
    "success": true,
    "message": ""
  },
  "result": true,
  "id": "<optional_string>"
}
````

##### 2.2.8 切换地图 

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

### 第五章回充管理

#### 5.1设置回冲点 

**Service**: `/cmd_dock` 
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/cmd_dock",
  "args": {
    "data": "set_dock_pose"
  },
  "id": "<optional_string>"
}

`````

响应：

`````
{
  "op": "service_response",
  "service": "/cmd_dock",
  "values": {
    "success": bool,
    "message": "<string>"
  },
  "result": bool,
  "id": "<optional_string>"
}

`````



#### 5.2返回回充点充电 

**Service**: `/cmd_dock` 
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/cmd_dock",
  "args": {
    "data": "dock"
  },
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/cmd_dock",
  "values": {
    "success": true,
    "message": "<string>"
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 5.3中断充电 

**Service**: `/cmd_dock` 
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/cmd_dock",
  "args": {
    "data": "undock"
  },
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/cmd_dock",
  "values": {
    "success": bool,
    "message": " "
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 

#### 5.4中断去充电 

**Service**: `/cmd_dock` 
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/cmd_dock",
  "args": {
    "data": "cancle_dock"
  },
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/cmd_dock",
  "values": {
    "success": true,
    "message": "<string>"
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 5.5订阅充电状态

Topic: **/dock_state**
 Type: **std_msgs/msg/String**

**Service**: `/cmd_dock` 
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "subscribe",
  "topic": "/dock_state",
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "publish",
  "topic": "/dock_state",
  "msg": {
    "data": "string"
  }
  "id": "<optional_string>"
}
data字段内容包括：
	undock 未充电
	goto_dock_pose 正在去充电
	charging 充电中

`````

### 第六章 电机与系统控制

#### 6.1 设置电机速度模式

说明：将电机切换到速度控制模式

**Service**: `/set_motor_mode`
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/set_motor_mode",
  "args": {
    "data": "velocity"
  },
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/set_motor_mode",
  "values": {
    "success": true,
    "message": "<string>"
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 6.2 设置电机力矩模式（电机释放）

说明：将电机切换到力矩控制模式，可用于释放电机

**Service**: `/set_motor_mode`
**Service Type**: `aid_robot_msgs/srv/SetString`

请求：

`````
{
  "op": "call_service",
  "service": "/set_motor_mode",
  "args": {
    "data": "torque"
  },
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/set_motor_mode",
  "values": {
    "success": true,
    "message": "<string>"
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 6.3 关闭悬崖检测

说明：通过设置参数关闭本地代价地图中的安全层（悬崖检测）

**Service**: `/local_costmap/local_costmap/set_parameters`
**Service Type**: `rcl_interfaces/srv/SetParameters`

请求：

`````
{
  "op": "call_service",
  "service": "/local_costmap/local_costmap/set_parameters",
  "args": {
    "parameters": [
      {
        "name": "safty_layer.enabled",
        "value": {
          "type": 1,
          "bool_value": false
        }
      }
    ]
  },
  "id": "<optional_string>"
}
`````

说明：`value.type` 字段为参数类型枚举值，`1` 对应 `PARAMETER_BOOL`

响应：

`````
{
  "op": "service_response",
  "service": "/local_costmap/local_costmap/set_parameters",
  "values": {
    "results": [
      {
        "successful": true,
        "reason": ""
      }
    ]
  },
  "result": true,
  "id": "<optional_string>"
}
`````

### 第七章 里程计-激光标定

说明：在线标定里程计内参（左右轮半径、轮距）和激光雷达到里程计坐标系的外参。整体流程为：调用开始标定服务 → 切换到遥控模式 → 遥控机器人走 S 形路线采集数据 → 调用结束标定服务返回结果 → 退出遥控模式。标定过程中可通过状态话题监听标定进度。

#### 7.1 开始标定

说明：通知标定节点开始采集激光和轮速数据。调用成功后，标定节点会订阅 `/scan` 和 `/joint_states`，并在检测到机器人运动后开始缓存数据。

**Service**: `/start_calibration_odom_laser`
**Service Type**: `aid_robot_msgs/srv/StartCalibration`

请求（无参数）：

`````
{
  "op": "call_service",
  "service": "/start_calibration_odom_laser",
  "args": {},
  "id": "<optional_string>"
}
`````

响应：

`````
{
  "op": "service_response",
  "service": "/start_calibration_odom_laser",
  "values": {
    "success": true,
    "message": "Calibration started; waiting for robot motion."
  },
  "result": true,
  "id": "<optional_string>"
}
`````

#### 7.2 切换遥控模式

说明：开始标定后需要切换到遥控模式，前端通过摇杆控制机器人行走。详见第四章模式设置。

**Service**: `/mode_set`
**Service Type**: `aid_robot_msgs/srv/StatusChange`

请求：

`````
{
  "op": "call_service",
  "service": "/mode_set",
  "args": [
    {
      "action": "remote_control"
    }
  ],
  "id": "<optional_string>"
}
`````

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

#### 7.3 遥控机器人行走

说明：标定过程中通过遥控话题控制机器人低速行走 S 形路线，路线需包含足够的平移和旋转。详见 1.3 遥控控制。

**Topic**: `/cmd_vel_remote_ctrl`
**Message Type**: `geometry_msgs/msg/Twist`

`````
{
  "op": "publish",
  "topic": "/cmd_vel_remote_ctrl",
  "msg": {
    "linear": {
      "x": 0.3,
      "y": 0.0,
      "z": 0.0
    },
    "angular": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.3
    }
  },
  "id": "<optional_string>"
}
`````

#### 7.4 结束标定

说明：停止数据采集并执行标定算法。标定成功后结果会写入 YAML 文件，并在响应中返回标定参数。调用结束后应切换回 idle 模式。

**Service**: `/end_calibration_odom_laser`
**Service Type**: `aid_robot_msgs/srv/EndCalibration`

请求（无参数）：

`````
{
  "op": "call_service",
  "service": "/end_calibration_odom_laser",
  "args": {},
  "id": "<optional_string>"
}
`````

响应（成功）：

`````
{
  "op": "service_response",
  "service": "/end_calibration_odom_laser",
  "values": {
    "success": true,
    "message": "Calibration finished.",
    "wheel_radius_left": 0.05,
    "wheel_radius_right": 0.05,
    "wheel_separation": 0.3,
    "laser_x": 0.0,
    "laser_y": 0.0,
    "laser_yaw": 0.0,
    "sample_count": 120,
    "output_file": "/home/dongfang/code/haier-robot-ws/config/odom_laser_calibration.yaml"
  },
  "result": true,
  "id": "<optional_string>"
}
`````

响应（失败）：

`````
{
  "op": "service_response",
  "service": "/end_calibration_odom_laser",
  "values": {
    "success": false,
    "message": "Insufficient calibration data.",
    "wheel_radius_left": 0.0,
    "wheel_radius_right": 0.0,
    "wheel_separation": 0.0,
    "laser_x": 0.0,
    "laser_y": 0.0,
    "laser_yaw": 0.0,
    "sample_count": 0,
    "output_file": ""
  },
  "result": true,
  "id": "<optional_string>"
}
`````

字段说明：

| 字段               | 类型    | 描述                              |
| ------------------ | ------- | --------------------------------- |
| success            | bool    | 标定是否成功                      |
| message            | string  | 结果描述或失败原因                |
| wheel_radius_left  | float64 | 左轮半径（m）                     |
| wheel_radius_right | float64 | 右轮半径（m）                     |
| wheel_separation   | float64 | 轮距（m）                         |
| laser_x            | float64 | 激光到里程计坐标系外参 x（m）     |
| laser_y            | float64 | 激光到里程计坐标系外参 y（m）     |
| laser_yaw          | float64 | 激光到里程计坐标系外参 yaw（rad） |
| sample_count       | uint64  | 参与标定的同步样本数              |
| output_file        | string  | 标定结果保存的 YAML 文件路径      |

#### 7.5 退出遥控模式

说明：标定结束后切换回 idle 模式，释放机器人控制权。

**Service**: `/mode_set`
**Service Type**: `aid_robot_msgs/srv/StatusChange`

请求：

`````
{
  "op": "call_service",
  "service": "/mode_set",
  "args": [
    {
      "action": "idle"
    }
  ],
  "id": "<optional_string>"
}
`````

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

#### 7.6 标定状态订阅

说明：标定节点通过该话题发布标定任务状态，使用 latched（transient_local）QoS，新订阅者会立即收到最近一次状态。

**Topic**: `/odom_laser_calibration_status`
**Message Type**: `aid_robot_msgs/msg/AidTaskStatus`

`````
{
  "op": "subscribe",
  "topic": "/odom_laser_calibration_status",
  "type": "aid_robot_msgs/msg/AidTaskStatus",
  "compression": "cbor",
  "id": "<optional_string>"
}
````

接收的消息：

`````

{
  "op": "publish",
  "topic": "/odom_laser_calibration_status",
  "msg": {
    "status": 1,
    "task_type": 0
  },
  "id": "<optional_string>"
}

````
status 状态说明：

| status | 含义 |
| --- | --- |
| 0 | idle（空闲） |
| 1 | working（正在标定） |
| 2 | success（标定完成） |
| 3 | failed（标定失败） |
| 4 | suspend（暂停） |
| 5 | cancel（取消） |

#### 7.7 标定流程示例

1. 调用 `/start_calibration_odom_laser` 开始标定
2. 调用 `/mode_set` 切换到 `remote_control` 模式
3. 通过 `/cmd_vel_remote_ctrl` 遥控机器人走 S 形路线，采集足够数据
4. 调用 `/end_calibration_odom_laser` 结束标定并获取结果
5. 调用 `/mode_set` 切换回 `idle` 模式
6. 全程可订阅 `/odom_laser_calibration_status` 监听标定状态 
````

### 第八章 版本查询

说明:查询机器的版本信息，目前有hub板/mcu版/电源管理版等

#### 8.1 HUB 硬件版本查询 

**Service**: `/get_hub_version`
**Service Type**: `std_srvs/srv/Trigger`

**请求：**

```json
{
  "op": "call_service",
  "service": "/get_hub_version",
  "args": {}
}
```

**响应：**

```json
{
  "op": "service_response",
  "service": "/get_hub_version",
  "values": {
    "success": true,
    "message": "hw_ver: v1.0.0, sw_ver: v1.0.1"
  },
  "result": true
}
```

#### 8.2 mcu版本查询

**Service**: `/get_embedded_version`
**Service Type**: ``aid_robot_msgs/srv/SetString``

请求参数

| `data` 取值 | 说明 |
|-------------|------|
| `"mcu_board"` | 查询 MCU 主板版本 |
| `"electrical_board"` | 查询电源管理板版本 |

**请求：**

```json
{
  "op": "call_service",
  "service": "/get_embedded_version",
  "args": {
    "data": "mcu_board"
  }
}
```

**响应：**

```json
{
  "op": "service_response",
  "service": "/get_embedded_version",
  "values": {
    "success": true,
    "message": "hw_ver: v1.0.0, sw_ver: v1.0.1"
  },
  "result": true
}
```

#### 8.3 导航版本

**Service**: `/get_release_info`
**Service Type**: ``std_srvs/srv/Trigger``

**请求：**

```json
{
  "op": "call_service",
  "service": "/get_release_info",
  "args": {}
}
```

**响应：**

```json
{
  "op": "service_response",
  "service": "/get_release_info",
  "values": {
    "success": false,
    "message": "string is null, get version failed"
  },
  "result": true
}
```



#### 8.4 ROS OTA 版本

**Service**: `/get_ros_ota_version`
**Service Type**: ```std_srvs/srv/Trigger```

**请求：**
```json
{
  "op": "call_service",
  "service": "/get_ros_ota_version",
  "args": {}
}
```

**响应：**
```json
{
  "op": "service_response",
  "service": "/get_ros_ota_version",
  "values": {
    "success": true,
    "message": "v1.2.3"
  },
  "result": true
}
```

