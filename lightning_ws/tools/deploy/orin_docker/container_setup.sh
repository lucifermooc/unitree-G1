#!/usr/bin/env bash
# 容器 g1 创建后的一次性补充（root 执行，可重复执行）：依赖工作区里的文件、没法写进 Dockerfile 的东西。
# g1_docker.sh 新建容器时自动调用（up / recreate）；手动：docker exec -u root g1 bash /opt/G1/docker/container_setup.sh
#
# Pangolin 0.9.3：lightning 的 cmake/packages.cmake 硬依赖它（运行时 with_ui:=false 不用）。
# Thor 上同样是从 lightning thirdparty 里的 Pangolin-0.9.3.zip 编译装到 /usr/local。
set -eo pipefail
ZIP="${PANGOLIN_ZIP:-/opt/G1/deps_backup/lightning_thirdparty/Pangolin-0.9.3.zip}"

if [ -f /usr/local/lib/cmake/Pangolin/PangolinConfig.cmake ]; then
  echo "Pangolin already installed"
else
  [ -f "$ZIP" ] || { echo "missing $ZIP (copy lightning_thirdparty from Thor /opt/G1/deps_backup)" >&2; exit 1; }
  apt-get update
  apt-get install -y --no-install-recommends unzip libglew-dev libepoxy-dev libwayland-dev libxkbcommon-dev wayland-protocols
  rm -rf /var/lib/apt/lists/*
  tmp="$(mktemp -d)"
  unzip -q "$ZIP" -d "$tmp"
  src="$(find "$tmp" -maxdepth 2 -name CMakeLists.txt -printf '%h\n' | head -1)"
  cmake -S "$src" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_TESTS=OFF
  cmake --build "$tmp/build" -j"$(nproc)"
  cmake --install "$tmp/build"
  ldconfig
  rm -rf "$tmp"
  echo "Pangolin installed"
fi
