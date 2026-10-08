#!/usr/bin/env bash
#
# 打包成单文件可执行程序。
#
# SDK 已 vendor 在 src/litegrip/ 下（来源见 src/litegrip/VENDORED.md），所以
# --paths src 一条就把控制台和 SDK 都交给了 PyInstaller，不用再指第二个目录。
#
# 这些参数是必须的：
#
#   --collect-data litegrip      把 SDK 的 factory_calibration.json 与两份方向
#                                模板打进产物。SDK 用 dirname(litegrip.__file__)
#                                找它们，在 sys._MEIPASS 下也成立；缺了它，机器上
#                                只剩「未标定、拒绝运动」。控制台自检里有一条专门
#                                查这个文件在不在（package-data 只管 wheel）。
#   --collect-submodules litegrip  否则只有直接 import 到的模块进包。
#
# --exclude-module zenoh / eclipse_zenoh：SDK 声明了但从未 import 的依赖，本机
# 也没装；不打进产物，也省下 PyInstaller 找不到它时的告警。
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

cd "$HERE"

# build.sh 自己也需要能 import litegrip，否则 --collect-* 无从下手。
# LITEGRIP_SDK_PATH 指过去时，那份检出排在 src/ 前面，于是打的是它。
export PYTHONPATH="${LITEGRIP_SDK_PATH:+$LITEGRIP_SDK_PATH:}$HERE/src${PYTHONPATH:+:$PYTHONPATH}"

# ── 收尾 ─────────────────────────────────────────────────────────────────────
# 构建期写进源码树的东西一律在退出时清掉，失败退出也要清。
#
# 版本戳尤其要清：它一旦留下，之后每次源码运行都会拿着上一次构建的号自称，而那个
# 号里的 git hash 可能早就和工作区对不上了——版本读数于是变成一句谎话。
# tests/test_cli.py 用「源码运行报 +source」钉着这条。它是构建的输入，不是源码的
# 一部分；产物里已经有一份自己的副本，删掉这里的不影响任何东西。
STAMP="src/litegrip_studio/_version.py"
cleanup() {
    rm -f "$STAMP"
    if [ -n "${ENTRY_DIR:-}" ]; then
        rm -rf "$ENTRY_DIR"
    fi
}
trap cleanup EXIT

# ── 版本戳 ───────────────────────────────────────────────────────────────────
# base 号先问 git 最近的一个 v* tag，于是从 v0.8.2 构建出来的产物自称
# 0.8.2.<提交数>+g<sha>.<日期>，一眼能对上是哪个发布；没有 tag 才退回 version.py
# 的 BASE_VERSION（单一来源）。这里生成的都不是发布版本号，发布版本由
# semantic-release 按提交历史算，git tag 才是唯一事实来源。
#
# 「|| true」不能少：脚本在 pipefail 下运行，而这个仓库/目录可能根本没有 tag，
# 那样 git describe 返回非零，整个赋值就会把脚本打退。
BASE="$(git describe --tags --abbrev=0 --match 'v*' 2>/dev/null | sed 's/^v//' || true)"
if [ -z "$BASE" ]; then
    BASE="$(sed -n 's/^BASE_VERSION *= *"\([^"]*\)".*/\1/p' src/litegrip_studio/version.py | head -1)"
fi
BASE="${BASE:-0.0.0}"
if git rev-parse --git-dir >/dev/null 2>&1; then
    COUNT="$(git rev-list --count HEAD 2>/dev/null || echo 0)"
    SHORT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
    DIRTY=""
    [ -n "$(git status --porcelain 2>/dev/null)" ] && DIRTY=".dirty"
    GITVER="+g${SHORT}${DIRTY}"
else
    COUNT="0"
    GITVER="+nogit"
fi
FULLVER="${BASE}.${COUNT}${GITVER}.$(date +%Y%m%d)"

printf '# 自动生成 —— 由 build.sh 写入，构建结束即删除，不必提交。\n__version__ = "%s"\n' \
    "$FULLVER" > "$STAMP"
echo "版本号：$FULLVER"

# ── 入口 ─────────────────────────────────────────────────────────────────────
# 交给 PyInstaller 的是这个绝对导入的小文件，而不是 src/litegrip_studio/__main__.py：
# PyInstaller 把入口脚本当顶层模块执行，那份文件里的相对导入（from .cli import main）
# 到了那里就没有父包可依。
#
# 目录随机、文件名固定：入口脚本的名字会进产物（它就是 __main__，回溯里显示的名
# 字），随机名会让同一个 commit 两次构建出不同字节，也把 litegrip-entry-lrwBgA 这
# 种东西写进报错信息。随机的是目录，于是并发的两次构建仍不会互相踩。
ENTRY_DIR="$(mktemp -d -t litegrip-build-XXXXXX)"
ENTRY="$ENTRY_DIR/litegrip_studio_main.py"
cat > "$ENTRY" <<'PY'
from litegrip_studio.cli import main

raise SystemExit(main())
PY

echo "用 $PYTHON_BIN 打包 litegrip-studio …"
# 不要在这里用 exec：exec 会用 PyInstaller 换掉本 shell 进程，上面那个 EXIT trap
# 就再也不会执行，版本戳和临时目录于是一起留在磁盘上。让脚本挂着等它结束，
# 退出码照样是它的（set -e 负责失败即退），收尾才有着落。
"$PYTHON_BIN" -m PyInstaller \
    --onefile \
    --noconfirm \
    --name "litegrip-studio" \
    --paths src \
    --collect-submodules litegrip \
    --collect-data litegrip \
    --hidden-import litegrip_studio._version \
    --exclude-module zenoh \
    --exclude-module eclipse_zenoh \
    --exclude-module matplotlib \
    --exclude-module tkinter \
    --exclude-module PySide2 \
    --exclude-module PySide6 \
    "$ENTRY"
