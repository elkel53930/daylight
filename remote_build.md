# Raspberry Pi + Remote Build ServerによるArduino開発環境

## 目的

Raspberry
Piを開発環境の中心として使用しながら、Arduino/ESP32のビルド処理を高速なLinuxマシンへオフロードする。

開発端末は以下を想定する。

-   Linuxマシン
-   Androidタブレット
-   Windowsマシン

各端末からSSHでRaspberry Piへ接続し、Raspberry
Pi上でソースコードを編集する。

------------------------------------------------------------------------

## システム構成

``` text
Linux / Android / Windows
          │
          │ SSH
          ▼
┌─────────────────────┐
│ Raspberry Pi        │
│                     │
│ - code-server       │
│ - ソースコード編集    │
│ - Git                │
│ - Arduino CLI upload│
│ - シリアルモニタ      │
└──────────┬──────────┘
           │
           │ SSH / rsync
           ▼
┌─────────────────────┐
│ Linux Build Server  │
│                     │
│ - Arduino CLI       │
│ - Arduino Core      │
│ - ライブラリ          │
│ - 高速コンパイル       │
└─────────────────────┘
```

Raspberry Piに接続されたESP32への書き込みは、Raspberry Pi上で実行する。

``` text
Raspberry Pi
     │
     │ USB
     ▼
   ESP32
```

------------------------------------------------------------------------

# ビルド方式

ビルド時には、Linux Build Serverの利用可能性を確認する。

``` text
make build
     │
     ▼
Build ServerへSSH接続確認
     │
     ├── 成功
     │      │
     │      ▼
     │   ソースコードをrsync
     │      │
     │      ▼
     │   Build Serverでコンパイル
     │      │
     │      ▼
     │   ビルド成果物をRaspberry Piへ取得
     │
     └── 失敗
            │
            ▼
       Raspberry Pi上でローカルビルド
```

Build
Serverが以下の状態でも、自動的にローカルビルドへフォールバックする。

-   電源OFF
-   ネットワークから切断
-   DHCPによるIPアドレス変更
-   mDNSによる名前解決失敗
-   SSHサーバー停止

------------------------------------------------------------------------

# Makefileのインターフェース

以下の3つのコマンドを提供する。

## `make build`

ビルドのみを実行する。

``` bash
make build
```

Build Serverが利用可能な場合：

``` text
Raspberry Pi
    │
    ├── ソースコード転送
    ▼
Linux Build Server
    │
    ├── Arduino CLI compile
    ▼
ビルド成果物
    │
    └── Raspberry Piへ転送
```

Build Serverが利用できない場合：

``` text
Raspberry Pi
    │
    └── Arduino CLI compile
```

------------------------------------------------------------------------

## `make upload`

ビルド後、ESP32へ書き込む。

``` bash
make upload
```

実行順序：

``` text
1. ビルド
   │
   ├── リモートビルド
   │   または
   └── ローカルビルド
   │
   ▼
2. Raspberry Pi上でESP32へ書き込み
```

------------------------------------------------------------------------

## `make upload-only`

ビルドせず、既存のビルド成果物をESP32へ書き込む。

``` bash
make upload-only
```

例えば：

``` bash
make build
make upload-only
```

とすることで、同じバイナリを複数回書き込める。

------------------------------------------------------------------------

# ネットワーク

## IPアドレスを直接指定しない

各マシンはDHCPでIPアドレスを取得するため、IPアドレスを設定ファイルへ直接記述しない。

代わりに、mDNSによるホスト名を使用する。

例：

``` text
Raspberry Pi       liner1.local
Linux Build Server build-pc.local
```

Raspberry PiからBuild Serverへ接続する場合：

``` bash
ssh user@build-pc.local
```

Makefileやスクリプトでも：

``` text
user@build-pc.local
```

を使用する。

------------------------------------------------------------------------

# SSH接続

Raspberry PiからBuild Serverへの接続は、SSH公開鍵認証を使用する。

## SSH鍵生成

Raspberry Pi上で：

``` bash
ssh-keygen -t ed25519
```

## 公開鍵登録

``` bash
ssh-copy-id user@build-pc.local
```

以後：

``` bash
ssh user@build-pc.local
```

でパスワードなしに接続できるようにする。

------------------------------------------------------------------------

# BUILD_HOSTの秘匿

Build Serverのホスト名・ユーザー名は、リポジトリが公開されている場合
GitHub上に公開したくない情報になりうる。`BUILD_HOST := user@build-pc.local`
をMakefileに直接書かず、`.gitignore`対象の`Makefile.local`に分離し、
Makefile側で`-include Makefile.local`する。

``` makefile
# Makefile側
-include Makefile.local
BUILD_HOST ?=
```

``` makefile
# Makefile.local（.gitignore対象、コミットしない）
BUILD_HOST := user@build-pc.local
```

リポジトリには実際の値を含まない`Makefile.local.example`だけを
コミットしておき、各開発者が手元でコピーして値を書き込む。
`BUILD_HOST`が空、またはファイルが存在しない場合は常にローカル
ビルドにフォールバックする。

------------------------------------------------------------------------

# Build Serverの可用性確認

単純なpingではなく、SSH接続そのものを確認する。

``` bash
ssh \
    -o ConnectTimeout=2 \
    -o BatchMode=yes \
    user@build-pc.local true
```

オプション：

-   `ConnectTimeout=2`
    -   接続待ち時間を2秒に制限する
-   `BatchMode=yes`
    -   パスワード入力などの対話を行わない
-   `true`
    -   接続確認だけを行う

成功した場合はリモートビルドを実行し、失敗した場合はローカルビルドへフォールバックする。

------------------------------------------------------------------------

# Arduino CLIの構成

## Raspberry Pi

主な役割：

-   ソースコード編集
-   Git
-   Arduino CLI
-   ESP32へのupload
-   シリアルモニタ

必要なArduino Coreとライブラリもインストールする。

``` bash
arduino-cli core install esp32:esp32
```

------------------------------------------------------------------------

## Linux Build Server

主な役割：

-   Arduino CLI compile
-   Arduino Core
-   Arduinoライブラリ
-   ビルドキャッシュ

例えばESP32-S3の場合：

``` bash
arduino-cli core install esp32:esp32
```

Build Server上で必要なライブラリもインストールする。

------------------------------------------------------------------------

# 推奨ディレクトリ構成

``` text
project/
├── Makefile
├── scripts/
│   └── arduino-build.sh
└── MySketch.ino
```

------------------------------------------------------------------------

# Makefile

``` makefile
# ========================================
# Configuration
# ========================================

SKETCH_DIR := .

FQBN := esp32:esp32:esp32s3
PORT ?= /dev/ttyUSB0
BAUD ?= 3000000

BUILD_HOST := user@build-pc.local
REMOTE_BASE_DIR := ~/arduino-remote-build

# リモート側のディレクトリ名は basename だけでは一意にならない。
# git worktree で複数ブランチを同時にチェックアウトすると、どの
# worktree でも "software/mob" のようにディレクトリ名が同じになり、
# 別々の worktree から make build すると同じリモートパスを取り合って
# しまう（rsync --delete と組み合わさっているため、片方のソースを
# もう片方が上書きしうる）。絶対パスのハッシュを付与して worktree
# ごとに別ディレクトリへ分離する。
#
# ただし arduino-cli はスケッチディレクトリ名と .ino ファイル名の
# 一致を要求する（実測確認: ディレクトリ名を "mob-<hash>" のように
# リネームすると "main file missing from sketch" で失敗する）。
# そのためハッシュは親ディレクトリに付け、リーフのディレクトリ名は
# 元の basename（"mob" 等）のまま保つ。
SKETCH_ABS_PATH := $(abspath $(SKETCH_DIR))
SKETCH_HASH     := $(shell echo -n "$(SKETCH_ABS_PATH)" | md5sum | cut -c1-8)
REMOTE_SKETCH_DIR := $(REMOTE_BASE_DIR)/$(SKETCH_HASH)/$(notdir $(SKETCH_ABS_PATH))

# --export-binaries の出力先は "<sketch>/build/<fqbn>/"
# （FQBNの ':' は '.' に置き換わる。例: esp32:esp32:esp32s3 →
# esp32.esp32.esp32s3）。実際に arduino-cli 1.5.1 で確認済み。
FQBN_DOTTED := $(subst :,.,$(FQBN))
EXPORT_DIR  := $(SKETCH_DIR)/build/$(FQBN_DOTTED)

# ========================================
# Targets
# ========================================

.PHONY: all build upload upload-only monitor clean

all: build

# ----------------------------------------
# Build
# ----------------------------------------

build:
    @./scripts/arduino-build.sh \
        "$(SKETCH_DIR)" \
        "$(FQBN)" \
        "$(BUILD_HOST)" \
        "$(REMOTE_SKETCH_DIR)"

# ----------------------------------------
# Build + Upload
# ----------------------------------------

# --input-dir は省略せず明示する。省略した場合の自動探索
# （--export-binaries の慣例ディレクトリを暗黙に見つける挙動）に
# 依存すると、実際に mob.ino の Makefile で「直前のビルドと無関係な
# 古いキャッシュが書き込まれる」事故が起きた前例があるため。
upload: build
    @arduino-cli upload \
        -p "$(PORT)" \
        --fqbn "$(FQBN)" \
        --input-dir "$(EXPORT_DIR)" \
        "$(SKETCH_DIR)"

# ----------------------------------------
# Upload existing binaries only
# ----------------------------------------

upload-only:
    @arduino-cli upload \
        -p "$(PORT)" \
        --fqbn "$(FQBN)" \
        --input-dir "$(EXPORT_DIR)" \
        "$(SKETCH_DIR)"

# ----------------------------------------
# Serial monitor
# ----------------------------------------

monitor:
    @arduino-cli monitor --port "$(PORT)" --config baudrate=$(BAUD)

# ----------------------------------------
# Clean
# ----------------------------------------

clean:
    @rm -rf "$(SKETCH_DIR)/build"
```

> Makefileのコマンド行は、スペースではなくTab文字でインデントする。

------------------------------------------------------------------------

# リモート/ローカルビルドスクリプト

`./scripts/arduino-build.sh`：

``` bash
#!/bin/bash

set -e

SKETCH_DIR="$1"
FQBN="$2"
BUILD_HOST="$3"
REMOTE_SKETCH_DIR="$4"

if [ -z "$SKETCH_DIR" ] ||
   [ -z "$FQBN" ] ||
   [ -z "$BUILD_HOST" ] ||
   [ -z "$REMOTE_SKETCH_DIR" ]; then
    echo "Usage:"
    echo "  $0 <sketch-dir> <fqbn> <build-host> <remote-sketch-dir>"
    exit 1
fi

echo "Checking build server: ${BUILD_HOST}"

if ssh \
    -o ConnectTimeout=2 \
    -o BatchMode=yes \
    "$BUILD_HOST" true 2>/dev/null
then
    echo "Build server is available."
    echo "Building remotely."

    ssh "$BUILD_HOST" \
        mkdir -p "$REMOTE_SKETCH_DIR"

    # build/ を除外しないと、前回のローカル/リモートビルド成果物を
    # 往復させるだけの無駄な転送になる（実害は小さいが、worktree間で
    # 混同した古い成果物を送り込む要因にもなりうる）。
    rsync -az --delete \
        --exclude 'build/' \
        "$SKETCH_DIR/" \
        "$BUILD_HOST:$REMOTE_SKETCH_DIR/"

    ssh "$BUILD_HOST" \
        arduino-cli compile \
        --fqbn "$FQBN" \
        --export-binaries \
        "$REMOTE_SKETCH_DIR"

    mkdir -p "$SKETCH_DIR/build"

    rsync -az \
        "$BUILD_HOST:$REMOTE_SKETCH_DIR/build/" \
        "$SKETCH_DIR/build/"

    echo "Remote build completed."

else
    echo "Build server is unavailable."
    echo "Building locally."

    arduino-cli compile \
        --fqbn "$FQBN" \
        --export-binaries \
        "$SKETCH_DIR"

    echo "Local build completed."
fi
```

実行権限を付与する。

``` bash
chmod +x scripts/arduino-build.sh
```

------------------------------------------------------------------------

# コマンド使用例

## 通常のビルド

``` bash
make build
```

Build Serverが起動していれば、Build Serverでコンパイルする。

Build Serverが見つからなければ、Raspberry Pi上でコンパイルする。

------------------------------------------------------------------------

## ビルドしてESP32へ書き込む

``` bash
make upload
```

以下を自動的に実行する。

``` text
make build
    │
    ▼
ESP32へupload
```

------------------------------------------------------------------------

## 既存のビルド成果物をESP32へ書き込む

``` bash
make upload-only
```

このコマンドではコンパイルを実行しない。

------------------------------------------------------------------------

## シリアルモニタ

``` bash
make monitor
```

ESP32からのシリアル出力を表示する（Ctrl+Cで終了）。ビルド・書き込みは
行わない。

------------------------------------------------------------------------

## クリーン

``` bash
make clean
```

Raspberry Pi上のビルド成果物を削除する。

------------------------------------------------------------------------

# 開発時の操作例

通常の開発フロー：

``` text
1. code-serverで編集
       │
       ▼
2. make build
       │
       ├── Build Serverあり
       │      └── 高速リモートビルド
       │
       └── Build Serverなし
              └── Raspberry Piでローカルビルド
       │
       ▼
3. make upload
       │
       ▼
4. ESP32で動作確認
```

同じビルド成果物を再度書き込む場合：

``` bash
make upload-only
```

------------------------------------------------------------------------

# 設計上の方針

## Raspberry Piを開発環境の中心にする

各端末から直接Build Serverへ接続するのではなく、必ずRaspberry
PiへSSHする。

``` text
Android ──┐
Windows ──┼── SSH ──▶ Raspberry Pi
Linux ────┘                 │
                            │ SSH
                            ▼
                       Linux Build Server
```

これにより、接続する端末に依存せず、開発環境を一貫させる。

------------------------------------------------------------------------

## Build Serverはオプション扱いにする

Build Serverは高速化のための補助的な存在とする。

``` text
Build Serverあり
    └── 高速ビルド

Build Serverなし
    └── Raspberry Piでビルド
```

Build Serverが停止していても開発作業を継続できることを優先する。

------------------------------------------------------------------------

## IPアドレスに依存しない

DHCPによってIPアドレスが変化しても動作するように、ホスト名を使用する。

``` text
user@build-pc.local
```

を使用し、以下のようなIPアドレスの直接指定は避ける。

``` text
user@192.168.1.100
```

------------------------------------------------------------------------

# 将来の改善候補

## 設定ファイルの分離

現在はMakefileに以下を直接記述している。

``` makefile
FQBN := esp32:esp32:esp32s3
PORT := /dev/ttyUSB0
BUILD_HOST := user@build-pc.local
```

将来的には、プロジェクトごとの設定ファイルに分離できる。

例：

``` text
arduino.yaml
```

``` yaml
fqbn: esp32:esp32:esp32s3
port: /dev/ttyUSB0
build_host: user@build-pc.local
```

------------------------------------------------------------------------

## 複数のBuild Server

将来的には複数のビルドサーバーを候補にして、最初に接続できたサーバーを利用することもできる。

``` text
build-pc-1.local
        │
        ├── 使用可能 → 使用
        │
        └── 使用不可
                │
                ▼
build-pc-2.local
        │
        ├── 使用可能 → 使用
        │
        └── 使用不可
                │
                ▼
          ローカルビルド
```

------------------------------------------------------------------------

# 最終的なユーザーインターフェース

開発者が意識するコマンドは以下だけにする。

``` bash
make build
```

``` bash
make upload
```

``` bash
make upload-only
```

Build
Serverの存在や、現在どこでビルドされているかを、通常の開発操作から隠蔽する。
