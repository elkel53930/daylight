FQBN        := esp32:esp32:esp32s3
SKETCH      := mob.ino
SKETCH_DIR  := .
PORT        ?= /dev/ttyUSB0
BAUD        ?= 3000000

BUILD_PATH  := .build

# リモートビルドサーバーの設定はリポジトリに含めない(ホスト名・
# ユーザー名を公開したくないため)。Makefile.local(.gitignore対象、
# Makefile.local.example参照)に BUILD_HOST := user@host.local と
# 書けば有効になる。無ければ常にローカルビルド。
-include Makefile.local
BUILD_HOST ?=

REMOTE_BASE_DIR := ~/arduino-remote-build
# 複数の git worktree で同じ "mob" ディレクトリ名を同時にビルドしても
# 衝突しないよう、絶対パスのハッシュでリモート側の親ディレクトリを
# worktree ごとに分離する。ただし arduino-cli はスケッチディレクトリ名
# と .ino ファイル名の一致を要求するため(実測確認: リネームすると
# "main file missing from sketch" で失敗する)、リーフのディレクトリ名
# は "mob" のまま保つ。
SKETCH_ABS_PATH   := $(abspath $(SKETCH_DIR))
SKETCH_HASH       := $(shell echo -n "$(SKETCH_ABS_PATH)" | md5sum | cut -c1-8)
REMOTE_SKETCH_DIR := $(REMOTE_BASE_DIR)/$(SKETCH_HASH)/$(notdir $(SKETCH_ABS_PATH))

.PHONY: all build upload upload-only monitor clean clean-all

all: build

## コンパイルのみ（書き込みなし）。BUILD_HOST が設定・到達可能なら
## リモートビルドサーバーでコンパイルし、成果物だけ持ち帰る。
## 未設定・到達不可なら Raspberry Pi 上でローカルビルドする。
build:
	@./scripts/arduino-build.sh \
		"$(SKETCH_DIR)" "$(FQBN)" "$(BUILD_PATH)" \
		"$(BUILD_HOST)" "$(REMOTE_SKETCH_DIR)"

## コンパイル＆書き込み
## --input-dir を指定しないと arduino-cli のデフォルトキャッシュ(前回の
## ビルドとは無関係な古いバイナリ)が書き込まれることがあるため必須。
upload: build
	arduino-cli upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_PATH) $(SKETCH)

## 書き込みのみ（ビルド済みの $(BUILD_PATH) を使用、リビルドしない）
upload-only:
	arduino-cli upload --fqbn $(FQBN) --port $(PORT) --input-dir $(BUILD_PATH) $(SKETCH)

## シリアルモニタ（Ctrl+C で終了）
monitor:
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

## 差分ビルド用キャッシュのみ削除（コア・ライブラリキャッシュは保持）
clean:
	rm -rf $(BUILD_PATH)

## 全キャッシュを削除（フルビルドに戻す）
clean-all:
	rm -rf $(BUILD_PATH)
