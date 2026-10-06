# LightNav-0 推理服务端（G1 Jetson Thor 本机）

在 G1 的 Jetson Thor（`unitree@192.168.128.146`，JetPack 7.0 / R38.2.1，sm_110，122 GB 统一内存）
上本机跑 [LightNav-0](https://github.com/BoragoCode/LightNav-0) 的 `lightnav-serve`
（WebSocket，默认 `ws://127.0.0.1:8050`），给 `/home/unitree/vln` 的 VLN 桥接直连用。
这不是 colcon 包，脚本在 Thor 上执行。

| 文件 | 作用 |
|---|---|
| `env.sh` | 路径 / 端口变量，以及把 pip、HF、vLLM、Triton、CUDA 缓存都重定向到 `$LN_ROOT/.cache`（被其余脚本 source） |
| `install_server.sh` | 建 venv、装 wheel、取代码、下模型、自检。分步执行，可重复跑 |
| `start_server.sh` | 封装 `env -u PYTHONPATH ... repo/scripts/serve_thor.sh`，默认 `GPU_MEM_UTIL=0.3`、fp8_llm_only、`127.0.0.1:8050` |
| `stop_server.sh` | 封装 `serve_thor.sh --stop`，只停本端口，等进程退出、内存释放 |
| `bench_ws.py` | 回放帧（目录 / gif / 视频）测服务端 `latency_ms`、`timings_ms`、`visible`、`opos` |
| `extract_bag_frames.py` | 从 ROS 2 bag 的彩色话题按 4 Hz 抽帧成 PPM（只读 bag，在单独的 ROS shell 里跑） |

## Thor 上的布局（只写这两处）

```
/home/unitree/lightnav/          LN_ROOT
  .venv/                         python3.12 venv（--without-pip + pip wheel 引导）
  repo/                          LightNav-0 @ c6f40e3
  scripts/                       本目录的拷贝
  .cache/ .config/ logs/ wheels/
  repo/logs/server_8050.log      服务日志（serve_thor.sh 写）
/home/unitree/models/LightNav-0/ 检查点（HF LightOriginsHQ/LightNav-0 @ 826dc5f）
```

`serve_thor.sh` 自己还会用 `/tmp/torchinductor_unitree_port8050` 作 inductor 缓存。

## 安装

```bash
# 工作站上
scp -r lightnav_server unitree@192.168.128.146:/home/unitree/lightnav/scripts
# Thor 上（干净 shell，不要 source ROS）
mkdir -p /home/unitree/lightnav/logs
nohup bash /home/unitree/lightnav/scripts/install_server.sh \
    > /home/unitree/lightnav/logs/install.log 2>&1 &
tail -f /home/unitree/lightnav/logs/install.log
```

步骤：`venv repo torch vllm deps model verify`，单独跑某几步：`STEPS="model verify" bash install_server.sh`。
版本和源都在脚本开头，可以用环境变量覆盖。

| 组件 | 版本 | 来源 | 原因 |
|---|---|---|---|
| torch / torchvision | 2.10.0+cu130 / 0.25.0+cu130 | download.pytorch.org/whl/cu130 | 含 sm_110；CUDA 13 运行库以 nvidia-* wheel 形式带进 venv。这台 Thor 没装 CUDA toolkit（/usr 下没有 libcudart/cuBLAS/cuDNN），JAL 的 torch 要链接系统库，用不了 |
| vllm | 0.19.0+cu130 | pypi.jetson-ai-lab.io/sbsa/cu130 | sm_110 原生编译；官方 0.19.1 aarch64 wheel 没有 sm_110 SASS。LightNav 补丁涉及的源码与 0.19.1 逐字节相同 |
| nvidia-cutlass-dsl | 4.5.2 | aliyun | 必须锁定，否则解析到 dev 版，启动时报 `ThrMma` |
| transformers | 5.8.0 | aliyun | LightNav 锁定版本 |
| compressed-tensors | 0.15.0.1 | aliyun | 0.19.0 自带的 0.14.0.1 要求 transformers<5；0.15.0.1 是 0.19.1 的锁定版本 |

`pip check` 会报三条预期内的提示，都只是元数据问题，不影响运行：
- vllm 0.19.0 的版本锁 `transformers<5`；
- vllm 0.19.0 的版本锁 `compressed-tensors==0.14.0.1`；
- `nvidia-cusparselt-cu13 0.8.0 is not supported on this platform`：这个 wheel 的 WHEEL 文件把标签写成了 `manylinux2014_sbsa`，库本身能正常加载。

模型：小文件走 `hf-mirror.com`。9.7 GB 的权重在 hf-mirror 上会 302 到 `cas-bridge.xethub.hf.co`，
这里只有约 30 KB/s，所以改从 ModelScope 同名仓库下载（约 7–9 MB/s）。下载前比对过：这份文件的 sha256
与 HF LFS 的 oid 一致（`ffc4a925…af18`），脚本下载完会再校验一次。

## 启动 / 停止

```bash
bash /home/unitree/lightnav/scripts/start_server.sh            # fp8_llm_only，等到 READY 才返回
QUANT=bf16 bash /home/unitree/lightnav/scripts/start_server.sh # bf16
HOST=0.0.0.0 bash .../start_server.sh                          # 对局域网开放（服务端没有鉴权）
bash /home/unitree/lightnav/scripts/stop_server.sh
```

- `GPU_MEM_UTIL` 默认 0.3，脚本拒绝 >0.30：机器人栈和语义地图也用这块统一内存。
- 可以从 source 过 ROS 的 shell 调用：脚本会去掉 `PYTHONPATH` / `LD_LIBRARY_PATH`。
- 同一端口只允许一个实例：`serve_thor.sh` 启动前会先等旧进程退出。

## 测试

```bash
cd /home/unitree/lightnav/repo
PY=/home/unitree/lightnav/.venv/bin/python
# 官方客户端
PYTHONPATH=src $PY -m lightnav.cli.ws_client --server ws://127.0.0.1:8050 \
    --frames /home/unitree/lightnav/test_frames/mujoco_demo_4hz \
    --instruction "move forward, then go to the trashcan on the right"
# 延迟 / timings / opos 统计
$PY /home/unitree/lightnav/scripts/bench_ws.py --gif docs/assets/mujoco_demo.gif \
    --crop 20,112,730,510 --fps 4 --episodes 2 \
    --instruction "move forward, then go to the trashcan on the right"
```

## VLN 桥接改连本机

`/home/unitree/vln/vln_ros2_bridge.py` 的 `--server` 默认值写死成 `ws://192.168.110.204:8050`（第 1588 行）。
不改文件，启动桥接时传参即可：`--server ws://127.0.0.1:8050`。
