# rosbridge v2.0 协议规范

> 本文档描述了 ROS 2 版本的 rosbridge。关于 ROS 1，请参见 [ros1 分支](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md)。

本文档概述了 rosbridge v2.0 协议。v2.0 协议整合了自 rosbridge 第一个版本发布以来出现的许多需求，并做了少量修改以促进协议更大的可扩展性。其核心是，协议仍然包含与先前版本 rosbridge 相同语义的相同操作。主要变化在于消息的结构，将控制信息与消息信息分离开来。主要的新增内容是分片、压缩和日志记录。

本文档概述了协议规范，但也涉及了 rosbridge 服务器实现的预期方向。rosbridge v2.0 服务器实现的方式使得添加和修改协议操作变得容易。此外，rosbridge v2.0 服务器将 JSON 处理与 websockets 服务器解耦，允许用户任意更改他们正在使用的特定 websockets 服务器实现。

rosbridge 的消息传输采用 JSON 对象。唯一必需的字段是 'op' 字段，它指定了该消息的操作。每个 'op' 然后指定其自己的消息语义。

rosbridge 协议是一组 'op' 代码，它们定义了许多操作以及每个操作的语义。

rosbridge 服务器是一个接受 websockets 连接并实现 rosbridge 协议的服务器。

rosbridge 的完整源代码位于 rosbridge_suite 包中。该包位于 https://github.com/robotwebtools/rosbridge_suite，并且该套件及其包的完整分解在本文档的 4.5 节中有详细说明。

## 1. rosbridge 传输

一个 rosbridge 消息，在基本情况下，是一个带有名为 "op" 的字符串字段的 JSON 对象。例如：

```json
{ "op": "Example" }
```

op 字段指示了此消息的类型。具有不同 op 值的消息可能会被不同地处理。

只要消息是一个带有 op 字段的 JSON 对象，它就是有效的 rosbridge 消息。

可选地，消息还可以提供一个任意的字符串或整数 ID：

```json
{ "op": "Example",
  "id": "fred"
}
```

如果向服务器提供的消息带有 ID，那么相关的响应消息通常也会包含该 ID。由此操作引起的日志消息也将包含该 ID。

在语义上，ID 不是它所在特定消息的标识符，而是一个交互的标识符，该交互可能由许多来回消息中的操作组成。因此，ID 可能被多个引用同一事务的消息使用。

## 2. rosbridge 协议

rosbridge 协议定义了许多不同的操作。它们如下：

消息压缩/转换：

  * **fragment** - 分片消息的一部分
  * **png** - PNG 压缩分片消息的一部分

Rosbridge 状态消息：

  * **set_status_level** - 设置 rosbridge 状态消息报告级别的请求
  * **status** - 状态消息

ROS 操作：

  * 主题 (Topics):
    * **advertise** – 通告您将发布一个主题
    * **unadvertise** – 停止通告您将发布主题
    * **publish** - 发布的 ROS 消息
    * **subscribe** - 订阅主题的请求
    * **unsubscribe** - 取消订阅主题的请求
  * 服务 (Services):
    * **advertise_service** - 通告一个外部服务服务器
    * **unadvertise_service** - 取消通告一个外部服务服务器
    * **call_service** - 服务调用
    * **service_response** - 服务响应
  * 动作 (Actions):
    * **advertise_action** - 通告一个外部动作服务器
    * **unadvertise_action** - 取消通告一个外部动作服务器
    * **send_action_goal** - 发送到动作服务器的目标
    * **cancel_action_goal** - 取消一个正在进行的动作目标
    * **action_feedback** - 来自动作服务器的反馈消息
    * **action_result** - 动作结果

通常，客户端采取的动作或操作（如发布和订阅）的操作码是动词（subscribe, call_service, unadvertise 等）。

来自服务器的响应消息是客户端返回的内容，因此它们是名词（fragment, status, service_response 等）。

（此命名约定的唯一轻微例外是 publish）

## 3. rosbridge 协议详情

以下是 rosbridge 协议中操作的规范，由 rosbridge 服务器支持。标记为 [实验性] 的内容在审查后可能会发生更改。

### 3.1 数据编码和转换

rosbridge 协议提供了分片消息和压缩消息的能力。

#### 3.1.1 分片 ( _fragment_ ) [实验性]

如果消息特别大，或者客户端请求分片，则可以对消息进行分片。分片的消息具有以下格式：

```json
{ "op": "fragment",
  "id": <string>,
  "data": <string>,
  "num": <int>,
  "total": <int>
}
```

**id** - 分片消息需要一个 id，以便识别分片消息对应的分片：

 * **data** - 数据的一个片段，当与其他数据片段组合时，构成另一个消息
 * **num** - 该片段在消息中的索引
 * **total** - 分片的总数

要对消息进行分片，需要获取其 JSON 字符串并将其拆分为多个子字符串。对于每个子字符串，构造一个分片消息，其中分片的数据字段由该子字符串填充。

要重建原始消息，需要将分片的数据字段连接起来，得到原始消息的 JSON 字符串。

#### 3.1.2 PNG 压缩 ( _png_ ) [实验性]

某些消息（如图像和地图）可能非常大，出于效率原因，我们可能希望将它们作为 PNG 编码的字节传输。PNG 操作码复制了 FRG 操作码的分片逻辑（并且可能且合理的是只有一个分片），不同之处在于数据字段由 ASCII 编码的 PNG 字节组成。

```json
{ "op": "png",
  (可选) "id": <string>,
  "data": <string>,
  (可选) "num": <int>,
  (可选) "total": <int>
}
```

 * **id** – 仅当消息被分片时需要。标识分片消息的分片。
 * **data** – PNG 编码消息的一个片段或整个消息。
 * **num** – 仅当消息被分片时需要。分片的索引。
 * **total** – 仅当消息被分片时需要。分片的总数。

要构造 PNG 压缩消息，请获取原始消息的 JSON 字符串，并将字符串的字节读入 PNG 图像。然后，对图像进行 ASCII 编码。此字符串现在用作数据字段。如果需要分片，则对数据进行分片，并在分片中设置适当的 ID、num 和 total 字段值。否则这些字段可以省略。

#### 3.1.3 CBOR 编码 ( _cbor_ )

[CBOR](https://tools.ietf.org/html/rfc7049) 编码是针对包含大数据块（如字节数组和数值类型数组）的消息最快的压缩方法。

当订阅者请求 CBOR 压缩时，将产生二进制消息而不是 JSON 字符串。一旦解码，消息将包含一个正常的协议消息。

该实现使用 [草案类型化数组标签] 来高效打包同构数组。目前，仅支持小端序打包。

[草案类型化数组标签]: https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00

#### 3.1.4 CBOR-RAW 编码 ( _cbor-raw_ )

虽然 CBOR 将整个消息编码为 CBOR，但有时希望获取 [ROS 序列化格式](https://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes) 中的原始二进制消息，该格式与 ROS 节点之间发送和存储在 [Bag 文件](http://wiki.ros.org/Bags/Format/2.0) 中的格式相同。

这在几种情况下可能有用：
- 您的应用程序已经知道如何解析 bag 文件中的消息（例如使用 [rosbag.js](https://github.com/cruise-automation/rosbag.js)），这意味着您现在可以对 bag 文件和实时消息使用一致的代码路径。
- 您希望尽可能晚地解析消息，或者并行解析，例如仅在关心消息的线程或 WebWorker 中解析。延迟消息的解析意味着将消息移动或复制到线程时，当其处于二进制形式时更便宜，因为不需要在线程之间进行序列化。
- 您只关心消息的一部分，不需要解析其余部分。
- 您非常关心性能；在 rosbridge_sever 中不进行 ROS 二进制格式和 CBOR 之间的转换。

该格式与上面的 CBOR 类似，但不是 "msg" 字段包含 CBOR 格式的消息本身，而是包含一个带有 "bytes" 字段的对象，该字段是一个包含原始消息的字节数组。"msg" 对象还包括接收到消息时 `get_rostime()` 的 "secs" 和 "nsecs"，这在设置了 `use_sim_time` 时特别有用，因为它将给出接收到消息的模拟时间。

使用此编码时，客户端应用程序需要确切知道如何解析原始消息。为此，使用 `/rosapi/get_topics_and_raw_types` 服务非常有用，该服务将提供所有主题及其原始消息定义，类似于 `gendeps --cat`。这与 bag 文件使用的格式相同。

### 3.2 状态消息

rosbridge 向客户端发送与 rosbridge 协议命令的成功和失败相关的状态消息。有四种状态级别：info、warning、error、none。默认情况下，rosbridge 使用 error 状态级别。

导致各级状态消息的粗略指南：

 * **error** – 当用户发送无效消息或请求不存在的内容时（即，发送不正确的操作码或发布到不存在的主题）
 * **warning** – error，加上，当用户做了可能成功但用户仍然做了不正确的事情时（即，提供部分完整的发布消息）
 * **info** – warning，加上指示各种操作成功的消息

#### 3.2.1 设置状态级别 ( _status_level_ ) [实验性]

```json
{ "op": "set_level",
  (可选) "id": <string>,
  "level": <string>
}
```

 * **level** – 'info', 'warning', 'error', 或 'none' 之一

将状态级别设置为指定的级别。如果指定了错误的字符串，则丢弃该消息。

#### 3.2.2 状态消息 ( _status_ ) [实验性]

```json
{ "op": "status",
  (可选) "id": <string>,
  "level": <string>,
  "msg": <string>
}
```

 * **level** – 此状态消息的级别
 * **msg** – 正在记录的字符串消息
 * **id** – 如果状态消息是某个具有 id 的操作的结果，则包含该 id

### 3.3 ROS 消息

这些 rosbridge 消息与 ROS 交互，并且大致对应于当前版本 rosbridge 中已经存在的消息。

#### 3.3.1 通告 ( _advertise_ )

如果您希望通告您正在或将要发布一个主题，请使用 advertise 命令。

```json
{ "op": "advertise",
  (可选) "id": <string>,
  "topic": <string>,
  "type": <string>
}
```

 * **topic** – 要通告的主题的字符串名称
 * **type** – 为主题通告的字符串类型

   * 如果主题尚不存在，并且指定的类型是有效类型，则将以该类型建立主题。
   * 如果主题已存在但具有不同的类型，则发送错误状态消息并丢弃此消息。
   * 如果主题已存在且具有相同的类型，则此消息的发送者被注册为另一个发布者。
   * 如果主题尚不存在但类型无法解析，则发送错误状态消息并丢弃此消息。

#### 3.3.2 取消通告 ( _unadvertise_ )

这将停止通告您正在发布一个主题。

```json
{ "op": "unadvertise",
  (可选) "id": <string>,
  "topic": <string>
}
```

 * **topic** – 正在取消通告的主题的字符串名称

   * 如果主题不存在，则发送警告状态消息并丢弃此消息
   * 如果主题存在并且仍有客户端在通告它，rosbridge 将继续通告它，直到所有客户端都取消通告
   * 如果主题存在但 rosbridge 没有通告它，则发送警告状态消息并丢弃此消息

#### 3.3.3 发布 ( _publish_ )

发布消息用于在主题上发送数据。

```json
{ "op": "publish",
  (可选) "id": <string>,
  "topic": <string>,
  "msg": <json>
}
```

publish 命令在主题上发布消息。

 * **topic** - 要发布到的主题的字符串名称
 * **msg** - 要在主题上发布的消息

   * 如果主题不存在，则发送错误状态消息并丢弃此消息
   * 如果 msg 不符合主题的类型，则发送错误状态消息并丢弃此消息
   * 如果 msg 是主题类型的子集，则发送警告状态消息，并使用默认值填充未指定的字段

特殊情况：如果要发布的类型具有 'header' 字段，则客户端可以选择从 msg 中省略 header。如果发生这种情况，rosbridge 将自动使用帧 id "" 和时间戳（当前时间）填充 header。或者，可以仅省略时间戳字段，然后将自动插入当前时间。

#### 3.3.4 订阅 ( _subscribe_ )

```json
{ "op": "subscribe",
  (可选) "id": <string>,
  "topic": <string>,
  (可选) "type": <string>,
  (可选) "throttle_rate": <int>,
  (可选) "queue_length": <int>,
  (可选) "fragment_size": <int>,
  (可选) "compression": <string>
}
```

此命令使客户端订阅指定主题。建议如果客户端有多个组件订阅同一主题，则每个组件应提供自己的 ID 进行订阅请求。这样，每个组件可以单独取消订阅，并且 rosbridge 可以选择适当的速率发送消息。

 * **type** – 要订阅主题的（预期）类型。如果省略，将推断类型，如果主题不存在，则订阅命令将失败
 * **topic** – 要订阅的主题的名称
 * **throttle_rate** – 发送消息之间必须经过的最短时间（以毫秒为单位）。默认为 0
 * **queue_length** – 用于缓冲消息的队列大小。消息由于 throttle_rate 而被缓冲。默认为 0（无队列）。
 * **id** – 如果指定，则可以通过引用 ID 来取消订阅此特定订阅。
 * **fragment_size** – 消息在分片前可以达到的最大大小。
 * **compression** – 一个可选字符串，用于指定在消息上使用的压缩方案。有效值为 "none"、"png"、"cbor" 和 "cbor-raw"。

如果指定了 queue_length，则消息在发送前被放入队列。消息从队列头部发送。如果队列已满，则最旧的消息被移除并由最新的消息替换。

如果一个客户端对同一主题有多个订阅，则消息以最低的 throttle_rate、最低的分片大小和最高的 queue_length 发送。建议客户端为其订阅提供 ID，以使 rosbridge 能够有效选择适当的分片大小和发布速率。

#### 3.3.5 取消订阅 ( _unsubscribe_ )

```json
{ "op": "unsubscribe",
  (可选) "id": <string>,
  "topic": <string>
}
```

 * **topic** – 要取消订阅的主题的名称
 * **id** – 要取消的订阅的 id

如果提供了 id，则仅取消订阅相应的订阅。如果未提供 ID，则取消所有订阅。

#### 3.3.6 通告服务 ( _advertise_service_ )

```json
{ "op": "advertise_service",
  "type": <string>,
  "service": <string>
}
```

通告一个外部 ROS 服务服务器。请求通过 Call Service 到达客户端。

 * **service** – 要通告的服务的名称
 * **type** – 通告的服务消息类型

#### 3.3.7 取消通告服务 ( _unadvertise_service_ )

```json
{ "op": "unadvertise_service",
  "service": <string>
}
```

#### 3.3.8 调用服务 ( _call_service_ )

调用一个 ROS 服务。

```json
{ "op": "call_service",
  (可选) "id": <string>,
  "service": <string>,
  (可选) "args": <list<json>>,
  (可选) "fragment_size": <int>,
  (可选) "compression": <string>,
  (可选) "timeout": <float>
}
```

 * **service** – 要调用的服务的名称
 * **args** – 如果服务没有参数，则不必提供 args，不过空列表同样可以接受。Args 应该是表示服务参数的 json 对象列表
 * **id** – 用于区分此服务调用的可选 id
 * **fragment_size** – 响应消息在分片前可以达到的最大大小
 * **compression** – 一个可选字符串，用于指定在消息上使用的压缩方案。有效值为 "none" 和 "png"
 * **timeout** – 等待服务器响应的超时时间，单位为秒

停止通告一个外部 ROS 服务服务器

 * **service** – 要取消通告的服务的名称

#### 3.3.9 服务响应 ( _service_response_ )

对 ROS 服务调用的响应。

```json
{ "op": "service_response",
  (可选) "id": <string>,
  "service": <string>,
  (可选) "values": <list<json>>,
  "result": <boolean>
}
```

 * **service** – 被调用的服务的名称
 * **values** – 返回值。如果服务没有返回值，则可以省略此字段（rosbridge 服务器会省略）
 * **id** – 如果服务请求提供了 ID，则服务响应将包含该 ID
 * **result** - 服务回调的返回值。true 表示成功，false 表示失败。

#### 3.3.10 通告动作 ( _advertise_action_ )

通告一个外部 ROS 动作服务器。

```json
{ "op": "advertise_action",
  "type": <string>,
  "action": <string>
}
```

 * **action** – 要通告的动作的名称
 * **type** – 通告的动作消息类型

#### 3.3.11 取消通告动作 ( _unadvertise_action_ )

停止通告一个外部 ROS 动作服务器。

```json
{ "op": "unadvertise_action",
  "action": <string>
}
```

 * **action** – 要取消通告的动作的名称

#### 3.3.12 发送动作目标 ( _send_action_goal_ )

发送一个动作目标。

```json
{ "op": "send_action_goal",
  (可选) "id": <string>,
  "action": <string>,
  (可选) "args": <list<json>>,
  (可选) "fragment_size": <int>,
  (可选) "compression": <string>
}
```

 * **action** – 要发送目标的动作的名称
 * **args** – 动作目标的参数。如果动作目标没有参数，则不必提供 args，不过空列表同样可以接受。Args 应该是表示动作目标参数的 json 对象列表
 * **id** – 用于区分此动作目标的可选 id
 * **fragment_size** – 响应消息在分片前可以达到的最大大小
 * **compression** – 一个可选字符串，用于指定在消息上使用的压缩方案。有效值为 "none" 和 "png"

#### 3.3.13 取消动作目标 ( _cancel_action_goal_ )

取消一个正在进行的动作目标。

```json
{ "op": "cancel_action_goal",
  (可选) "id": <string>,
  "action": <string>
}
```

 * **action** – 要取消目标的动作的名称
 * **id** – 要取消的动作目标的 id

#### 3.3.14 动作反馈 ( _action_feedback_ )

来自动作服务器的反馈消息。

```json
{ "op": "action_feedback",
  (可选) "id": <string>,
  "action": <string>,
  (可选) "values": <list<json>>
}
```

 * **action** – 发送反馈的动作的名称
 * **values** – 反馈值。如果动作没有反馈值，则可以省略此字段（rosbridge 服务器会省略）
 * **id** – 如果动作目标提供了 ID，则动作反馈将包含该 ID

#### 3.3.15 动作结果 ( _action_result_ )

动作结果消息。

```json
{ "op": "action_result",
  (可选) "id": <string>,
  "action": <string>,
  (可选) "values": <list<json>>,
  "result": <boolean>
}
```

 * **action** – 发送结果的动作的名称
 * **values** – 结果值。如果动作没有结果值，则可以省略此字段（rosbridge 服务器会省略）
 * **id** – 如果动作目标提供了 ID，则动作结果将包含该 ID
 * **result** - 动作回调的返回值。true 表示成功，false 表示失败。

### 3.4 其他操作

#### 3.4.1 认证 ( _auth_ )

```json
{ "op": "auth",
  "mac": <string>,
  "client": <string>,
  "dest": <string>,
  "rand": <string>,
  "t": <int>,
  "level": <string>,
  "end": <int>
}
```

 * **mac** – 消息认证码
 * **client** – 客户端的 IP 地址
 * **dest** – 目标机器的 IP 地址
 * **rand** – 随机字符串
 * **t** – 时间戳
 * **level** – 用户级别
 * **end** – 时间戳，此认证过期的时间

#### 3.4.2 获取 ROS 系统状态 ( _get_system_state_ )

```json
{ "op": "get_system_state",
  (可选) "id": <string>
}
```

 * **id** – 用于区分此系统状态请求的可选 id

响应：

```json
{ "op": "system_state",
  (可选) "id": <string>,
  "state": {
    "topics": [ [ <string>, <string> ] ],
    "services": [ <string> ],
    "actions": [ <string> ]
  }
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **state** – 包含系统状态的对象
   * **topics** – 一个列表，其中每个元素是 [topic_name, topic_type]
   * **services** – 一个服务名称列表
   * **actions** – 一个动作名称列表

#### 3.4.3 获取参数 ( _get_param_ )

```json
{ "op": "get_param",
  (可选) "id": <string>,
  "name": <string>,
  (可选) "default": <json>
}
```

 * **id** – 用于区分此参数请求的可选 id
 * **name** – 要获取的参数的名称
 * **default** – 如果参数不存在，则返回的默认值

响应：

```json
{ "op": "param_value",
  (可选) "id": <string>,
  "value": <json>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **value** – 参数的值

#### 3.4.4 设置参数 ( _set_param_ )

```json
{ "op": "set_param",
  (可选) "id": <string>,
  "name": <string>,
  "value": <json>
}
```

 * **id** – 用于区分此参数设置请求的可选 id
 * **name** – 要设置的参数的名称
 * **value** – 要设置的参数的值

响应：

```json
{ "op": "status",
  (可选) "id": <string>,
  "level": "info",
  "msg": "set"
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID

#### 3.4.5 删除参数 ( _delete_param_ )

```json
{ "op": "delete_param",
  (可选) "id": <string>,
  "name": <string>
}
```

 * **id** – 用于区分此参数删除请求的可选 id
 * **name** – 要删除的参数的名称

响应：

```json
{ "op": "status",
  (可选) "id": <string>,
  "level": "info",
  "msg": "deleted"
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID

#### 3.4.6 搜索参数 ( _search_param_ )

```json
{ "op": "search_param",
  (可选) "id": <string>,
  "name": <string>
}
```

 * **id** – 用于区分此参数搜索请求的可选 id
 * **name** – 要搜索的参数的名称

响应：

```json
{ "op": "search_result",
  (可选) "id": <string>,
  "value": <string>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **value** – 找到的参数的名称，如果未找到则为空字符串

#### 3.4.7 订阅参数 ( _subscribe_param_ )

```json
{ "op": "subscribe_param",
  (可选) "id": <string>,
  "name": <string>
}
```

 * **id** – 用于区分此参数订阅请求的可选 id
 * **name** – 要订阅的参数的名称

响应：

```json
{ "op": "param_update",
  (可选) "id": <string>,
  "name": <string>,
  "value": <json>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **name** – 已更新的参数的名称
 * **value** – 参数的新值

#### 3.4.8 取消订阅参数 ( _unsubscribe_param_ )

```json
{ "op": "unsubscribe_param",
  (可选) "id": <string>,
  "name": <string>
}
```

 * **id** – 用于区分此参数取消订阅请求的可选 id
 * **name** – 要取消订阅的参数的名称

响应：

```json
{ "op": "status",
  (可选) "id": <string>,
  "level": "info",
  "msg": "unsubscribed"
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID

### 3.5 其他操作

#### 3.5.1 获取操作列表 ( _get_operations_list_ )

```json
{ "op": "get_operations_list",
  (可选) "id": <string>
}
```

 * **id** – 用于区分此操作列表请求的可选 id

响应：

```json
{ "op": "operations_list",
  (可选) "id": <string>,
  "operations": [ <string> ]
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **operations** – 支持的操作列表

#### 3.5.2 获取主题类型 ( _get_topic_type_ )

```json
{ "op": "get_topic_type",
  (可选) "id": <string>,
  "topic": <string>
}
```

 * **id** – 用于区分此主题类型请求的可选 id
 * **topic** – 要获取其类型的主题的名称

响应：

```json
{ "op": "topic_type",
  (可选) "id": <string>,
  "type": <string>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **type** – 主题的类型，如果主题不存在则为空字符串

#### 3.5.3 获取服务类型 ( _get_service_type_ )

```json
{ "op": "get_service_type",
  (可选) "id": <string>,
  "service": <string>
}
```

 * **id** – 用于区分此服务类型请求的可选 id
 * **service** – 要获取其类型的服务的名称

响应：

```json
{ "op": "service_type",
  (可选) "id": <string>,
  "type": <string>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **type** – 服务的类型，如果服务不存在则为空字符串

#### 3.5.4 获取动作类型 ( _get_action_type_ )

```json
{ "op": "get_action_type",
  (可选) "id": <string>,
  "action": <string>
}
```

 * **id** – 用于区分此动作类型请求的可选 id
 * **action** – 要获取其类型的动作的名称

响应：

```json
{ "op": "action_type",
  (可选) "id": <string>,
  "type": <string>
}
```

 * **id** – 如果请求提供了 ID，则响应将包含该 ID
 * **type** – 动作的类型，如果动作不存在则为空字符串

## 4. 实现

### 4.1 服务器

rosbridge 服务器是一个 websockets 服务器，它接受 websockets 连接，然后通过 websockets 连接接收和发送 rosbridge 消息。服务器将 websockets 服务器与 rosbridge 协议处理解耦，以便可以轻松更改 websockets 服务器实现。

### 4.2 客户端

rosbridge 客户端是一个连接到 rosbridge 服务器的 websockets 客户端。客户端发送和接收 rosbridge 消息。

### 4.3 消息处理

rosbridge 服务器接收消息，解析 JSON，然后根据消息的 'op' 字段处理消息。对于每个操作，服务器调用一个处理程序函数。处理程序函数可以执行任何操作，包括发送消息。

### 4.4 消息发送

当服务器需要发送消息时，它构造一个 JSON 对象，将其字符串化，然后通过 websockets 连接发送。

### 4.5 源代码组织

rosbridge 套件由以下包组成：

 * **rosbridge_library** – 包含 rosbridge 协议实现的核心包。该包包含处理每个操作的处理程序，以及将 JSON 转换为 ROS 消息和将 ROS 消息转换为 JSON 的代码。
 * **rosbridge_server** – 包含 rosbridge 服务器的包。该包包含启动 websockets 服务器和将传入消息传递给 rosbridge_library 的代码。
 * **rosbridge_suite** – 一个元包，用于轻松安装整个 rosbridge 套件。

rosbridge 套件的源代码位于 https://github.com/robotwebtools/rosbridge_suite。

### 4.6 测试

rosbridge 套件包含一套测试，用于验证 rosbridge 协议的正确实现。测试位于 rosbridge_library 包中。

## 5. 参考文献

 * [rosbridge v2.0 协议规范](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
 * [rosbridge 套件](https://github.com/RobotWebTools/rosbridge_suite)
 * [ROS](https://www.ros.org/)
