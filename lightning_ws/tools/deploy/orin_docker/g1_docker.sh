#!/usr/bin/env bash
# G1 Orin（JetPack 5 / Ubuntu 20.04）上运行 jazzy 栈的常驻容器 g1。装在 /opt/G1/docker/（install_orin.sh），
# 不在工作区里：部署删 src 也不影响它。
# 容器只创建一次（--restart unless-stopped，开机随 docker 起来），之后 shell / 编译 / 开机服务都 docker exec 进同一个容器，
# 在容器里装的东西会保留。路径与 Thor 主机一致：/opt/G1、/home/unitree、/tmp 原样挂进去，
# host 网络 / pid / ipc，privileged（D435 USB、/dev）。所以 install 里的 g1_autostart.sh、dds_env.sh、g1_service.sh 不用改。
#
#   g1_docker.sh build-image              构建镜像 g1-jazzy:latest（Dockerfile 在本目录）
#   g1_docker.sh up                       容器 g1 不存在就创建，停着就启动（其他子命令会自动调用）
#   g1_docker.sh recreate                 重建镜像后用新镜像重建容器（容器里手动装的东西会丢）
#   g1_docker.sh shell                    进容器交互 shell（已 source jazzy + install + dds_env.sh，不读主机 ~/.bashrc）
#   g1_docker.sh run <命令...>            在容器里执行一条命令（同上环境），如 g1_docker.sh run ros2 topic list
#   g1_docker.sh service robot|semantic   systemd ExecStart：前台执行 g1_autostart.sh，进程号写 /tmp/g1-<名>.pid
#   g1_docker.sh service-stop robot|semantic  systemd ExecStop：给该进程发 SIGINT（= Ctrl-C），最多等 30 s
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${G1_IMAGE:-g1-jazzy:latest}"
NAME="${G1_CONTAINER:-g1}"
G1_WS="${G1_WS:-/opt/G1/lighting_ws}"
AUTOSTART="$G1_WS/install/robot_bringup/share/robot_bringup/script/g1_autostart.sh"

up() {
  local state; state="$(docker inspect -f '{{.State.Status}}' "$NAME" 2>/dev/null || true)"
  case "$state" in
    running) ;;
    "")
      docker run -d --name "$NAME" --restart unless-stopped --init -e TINI_SUBREAPER=1 \
        --net=host --pid=host --ipc=host --privileged \
        -v /dev:/dev -v /run/udev:/run/udev:ro -v /opt/G1:/opt/G1 -v /home/unitree:/home/unitree -v /tmp:/tmp \
        -v /etc/localtime:/etc/localtime:ro -e HOME=/home/unitree -e G1_WS="$G1_WS" -u unitree -w /home/unitree \
        "$IMAGE" sleep infinity >/dev/null
      echo "created container $NAME ($IMAGE)" >&2
      # 依赖工作区文件的补充安装（Pangolin 等），见 container_setup.sh
      docker exec -u root "$NAME" bash "$DIR/container_setup.sh" >&2 ;;
    *) docker start "$NAME" >/dev/null ;;
  esac
}

TTY=(); [ -t 0 ] && [ -t 1 ] && TTY=(-it)

case "${1:-}" in
  build-image)
    exec docker build -t "$IMAGE" "$DIR"
    ;;
  up)
    up
    ;;
  recreate)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    up
    ;;
  shell)
    up
    exec docker exec -it "$NAME" /bin/bash --rcfile /etc/g1_env.sh -i
    ;;
  run)
    shift
    [ $# -gt 0 ] || { echo "usage: $0 run <command...>" >&2; exit 2; }
    up
    exec docker exec "${TTY[@]}" "$NAME" \
      /bin/bash -c 'source /etc/g1_env.sh >/dev/null; exec "$@"' g1 "$@"
    ;;
  service)
    what="${2:?usage: $0 service robot|semantic}"
    up
    # --pid=host：容器里的 $$ 就是主机 pid，service-stop 直接在主机上 kill 它
    exec docker exec "$NAME" /bin/bash -c 'echo $$ > "/tmp/g1-$1.pid"; exec /bin/bash "$2" "$1"' g1 "$what" "$AUTOSTART"
    ;;
  service-stop)
    what="${2:?usage: $0 service-stop robot|semantic}"
    pidf="/tmp/g1-$what.pid"
    pid="$(cat "$pidf" 2>/dev/null || true)"
    # docker exec 不转发信号，所以由这里给容器里的进程发 SIGINT；之后 systemd 再停 docker exec 客户端
    if [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null; then
      for _ in $(seq 60); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -0 "$pid" 2>/dev/null && { echo "pid $pid still alive after 30 s, SIGKILL" >&2; kill -KILL "$pid" || true; }
    fi
    rm -f "$pidf"
    ;;
  *)
    sed -n '2,16p' "$0"; exit 2
    ;;
esac
