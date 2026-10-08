#!/usr/bin/env bash
#
# 启动脚本。所有路径都加引号——这个仓库的目录名里带空格。
#
# 主路径不安装 SDK：SDK 已经随本仓库发在 src/litegrip/ 下（来源见
# src/litegrip/VENDORED.md），把 src/ 放进 PYTHONPATH 就够了——不用 pip install，
# 也不用在别处再检出一份。它唯一的声明依赖 eclipse-zenoh 在库代码里根本没有
# import，所以连依赖都不用装。
#
# 要对着 SDK 的 HEAD 跑，用 LITEGRIP_SDK_PATH 指到那份检出，它排在 src/ 前面，
# 于是覆盖仓库里这份。
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

export PYTHONPATH="${LITEGRIP_SDK_PATH:+$LITEGRIP_SDK_PATH:}$HERE/src${PYTHONPATH:+:$PYTHONPATH}"

usage() {
    cat <<EOF
用法：$(basename "$0") [gui|sim|selftest|test|build] [附加参数…]

  gui       真机，通过 SocketCAN（等同于 --backend real）
  sim       仿真被控对象，不需要硬件
  selftest  不需要 Qt、不需要硬件的自检
  test      跑测试套件
  build     打包成单文件可执行程序

其余参数原样传给 litegrip-studio，例如：
  $(basename "$0") sim --can-channel can1
  $(basename "$0") gui --log-level DEBUG

当前解释器：$PYTHON_BIN
SDK 路径：  ${LITEGRIP_SDK_PATH:-$HERE/src/litegrip}${LITEGRIP_SDK_PATH:+（由 LITEGRIP_SDK_PATH 指定）}
EOF
}

case "${1:-gui}" in
    gui) exec "$PYTHON_BIN" -m litegrip_studio --backend real "${@:2}" ;;
    sim) exec "$PYTHON_BIN" -m litegrip_studio --backend sim "${@:2}" ;;
    selftest) exec "$PYTHON_BIN" -m litegrip_studio --selftest "${@:2}" ;;
    test) exec "$PYTHON_BIN" -m pytest "$HERE/tests" -q "${@:2}" ;;
    build) exec "$HERE/build.sh" ;;
    -h | --help | help) usage ;;
    *) echo "未知子命令：$1（用 -h 看用法）" >&2 && exit 2 ;;
esac
