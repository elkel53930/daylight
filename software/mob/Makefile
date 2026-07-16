FQBN        := esp32:esp32:esp32s3
SKETCH      := mob.ino
PORT        ?= /dev/ttyUSB0
BAUD        ?= 3000000

BUILD_PATH  := .build

COMPILE_FLAGS := --fqbn $(FQBN) \
                 --build-path $(BUILD_PATH)

.PHONY: all build upload monitor clean clean-all

all: build

## コンパイルのみ（書き込みなし）
build:
	arduino-cli compile $(COMPILE_FLAGS) $(SKETCH)

## コンパイル＆書き込み
upload:
	arduino-cli compile $(COMPILE_FLAGS) $(SKETCH)
	arduino-cli upload  --fqbn $(FQBN) --port $(PORT) $(SKETCH)

## シリアルモニタ（Ctrl+C で終了）
monitor:
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

## 差分ビルド用キャッシュのみ削除（コア・ライブラリキャッシュは保持）
clean:
	rm -rf $(BUILD_PATH)

## 全キャッシュを削除（フルビルドに戻す）
clean-all:
	rm -rf $(BUILD_PATH)
