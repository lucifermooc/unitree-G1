#!/usr/bin/env bash
# 在 G1 Orin 上安装容器版开机自启（以 unitree 用户执行，需要时 sudo）。对应 Thor 上的 g1_service.sh install。
#   bash install_orin.sh          拷 Dockerfile / g1_docker.sh 到 /opt/G1/docker，装两个 unit + sudoers + DDS 缓冲，
#                                 起 Qdrant 容器（restart always）；不构建镜像、不启动栈
# 之后：g1_docker.sh build-image → 全量编译（README.md）→ g1_service.sh restart。启停照常用 install 里的 g1_service.sh。
set -eo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST=/opt/G1/docker
UNITS=(g1-robot.service g1-semantic-map.service)
USER_NAME="$(id -un)"
[ "$USER_NAME" = root ] && { echo "run as unitree, not root" >&2; exit 1; }

mkdir -p "$DST" /opt/G1/logs
install -m 644 "$SRC/Dockerfile" "$SRC/g1_env.sh" "$DST/"
install -m 755 "$SRC/g1_docker.sh" "$DST/"
for u in "${UNITS[@]}"; do sudo install -m 644 "$SRC/$u" "/etc/systemd/system/$u"; done

# 与 g1_service.sh 的 sudoers_rule 相同：只放行这两个服务的 start/stop/restart 免密
sc="$(command -v systemctl)"; cmds=()
for verb in start stop restart; do
  for target in "${UNITS[*]}" "${UNITS[0]}" "${UNITS[1]}"; do cmds+=("$sc $verb $target"); done
done
tmp="$(mktemp)"
{ echo "# 由 install_orin.sh 生成：$USER_NAME 免密启停 G1 自启服务（仅限以下命令）"
  IFS=,; echo "$USER_NAME ALL=(root) NOPASSWD: ${cmds[*]}"; unset IFS; } > "$tmp"
sudo visudo -cf "$tmp" >/dev/null && sudo install -m 0440 -o root -g root "$tmp" /etc/sudoers.d/g1-autostart
rm -f "$tmp"

SYSCTL=/opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/system/60-dds-buffers.conf
[ -f "$SYSCTL" ] && sudo install -m 644 "$SYSCTL" /etc/sysctl.d/60-dds-buffers.conf
[ -f /etc/sysctl.d/60-dds-buffers.conf ] && sudo sysctl -p /etc/sysctl.d/60-dds-buffers.conf \
  || echo "WARN /etc/sysctl.d/60-dds-buffers.conf missing (build robot_bringup, re-run)" >&2

# 语义地图的向量库（Thor 上同名同版本容器）
if ! docker inspect qdrant >/dev/null 2>&1; then
  mkdir -p "$HOME/qdrant/storage"
  docker run -d --name qdrant --restart always -p 6333:6333 -p 6334:6334 \
    -v "$HOME/qdrant/storage:/qdrant/storage" qdrant/qdrant:v1.19.1
fi

sudo systemctl daemon-reload
sudo systemctl enable "${UNITS[@]}"
echo "installed ${UNITS[*]} (container mode), enabled at boot; not started."
