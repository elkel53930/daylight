#!/bin/bash
# ローカル/リモートビルド切り替えスクリプト。
#
# BUILD_HOST が設定されていて SSH で到達できれば、そのビルドサーバー上
# で arduino-cli compile を実行し、成果物(BUILD_PATH)だけを rsync で
# 持ち帰る。BUILD_HOST が未設定・到達できない場合はもちろん、SSHは
# 通るがリモート側の setup が不完全(arduino-cli 未導入・ESP32コア
# 未導入・rsync失敗等)でリモートビルドの途中で失敗した場合も、
# Raspberry Pi 上でのローカルビルドにフォールバックする
# (Build Server側の一時的・恒久的な不備で開発が止まらないようにする
# のが目的なので、SSH到達可否だけでなく実際のビルド成否で判断する)。
#
# 使い方:
#   arduino-build.sh <sketch-dir> <fqbn> <build-path> [build-host] [remote-sketch-dir]

set -e

SKETCH_DIR="$1"
FQBN="$2"
BUILD_PATH="$3"
BUILD_HOST="$4"
REMOTE_SKETCH_DIR="$5"

if [ -z "$SKETCH_DIR" ] || [ -z "$FQBN" ] || [ -z "$BUILD_PATH" ]; then
    echo "Usage:"
    echo "  $0 <sketch-dir> <fqbn> <build-path> [build-host] [remote-sketch-dir]"
    exit 1
fi

if [ -n "$BUILD_HOST" ] && [ -z "$REMOTE_SKETCH_DIR" ]; then
    echo "Error: build-host is set but remote-sketch-dir is empty."
    exit 1
fi

build_locally() {
    arduino-cli compile \
        --fqbn "$FQBN" \
        --build-path "$SKETCH_DIR/$BUILD_PATH" \
        "$SKETCH_DIR"
    echo "Local build completed."
}

# 各ステップで明示的に || return 1 する。「if (set -e; ...) 」のように
# set -e の伝播に頼る書き方は、if の条件式中では errexit が働かない
# bash の仕様のせいで実際にコンパイル失敗を見逃した(rsyncエラー・
# arduino-cli未導入のいずれも検知できず「Remote build completed」と
# 誤表示した実測あり)。終了コードは必ず自分でチェックする。
build_remotely() {
    ssh "$BUILD_HOST" mkdir -p "$REMOTE_SKETCH_DIR" || return 1

    # BUILD_PATH は毎回リモートで作り直すので、ローカル→リモートの
    # 同期対象からは除外する(往復させるだけ無駄になるうえ、worktree間
    # で混同した古い成果物を送り込む要因にもなりうる)。
    rsync -az --delete \
        --exclude "$BUILD_PATH/" \
        "$SKETCH_DIR/" \
        "$BUILD_HOST:$REMOTE_SKETCH_DIR/" || return 1

    # bare な "arduino-cli" ではなくフルパスで呼ぶ: ssh 経由のコマンドは
    # 非対話シェルで実行されるため、.bashrc 等での PATH 追加が反映され
    # ない(実測: x13u では ~/.local/bin/arduino-cli に存在。~/bin ではない
    # ので注意。ビルドサーバーが変わったら要確認)。
    ssh "$BUILD_HOST" \
        '~/.local/bin/arduino-cli' compile \
        --fqbn "$FQBN" \
        --build-path "$REMOTE_SKETCH_DIR/$BUILD_PATH" \
        "$REMOTE_SKETCH_DIR" || return 1

    mkdir -p "$SKETCH_DIR/$BUILD_PATH"
    rsync -az \
        "$BUILD_HOST:$REMOTE_SKETCH_DIR/$BUILD_PATH/" \
        "$SKETCH_DIR/$BUILD_PATH/" || return 1

    echo "Remote build completed."
}

if [ -n "$BUILD_HOST" ] && ssh -o ConnectTimeout=2 -o BatchMode=yes "$BUILD_HOST" true 2>/dev/null; then
    echo "Build server ($BUILD_HOST) is available. Building remotely."
    if build_remotely; then
        exit 0
    fi
    echo "Remote build failed. Falling back to local build." >&2
else
    if [ -z "$BUILD_HOST" ]; then
        echo "No build server configured (see Makefile.local.example). Building locally."
    else
        echo "Build server ($BUILD_HOST) is unavailable. Building locally."
    fi
fi

build_locally
