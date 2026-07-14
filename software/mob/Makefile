FQBN    := esp32:esp32:esp32s3
SKETCH  := mob.ino
PORT    ?= /dev/ttyUSB0
BAUD    ?= 3000000

.PHONY: all build upload monitor clean

all: build

## コンパイルのみ（書き込みなし）
build:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH)

## コンパイル＆書き込み
upload:
	arduino-cli compile --fqbn $(FQBN) $(SKETCH)
	arduino-cli upload  --fqbn $(FQBN) --port $(PORT) $(SKETCH)

## シリアルモニタ（Ctrl+C で終了）
monitor:
	arduino-cli monitor --port $(PORT) --config baudrate=$(BAUD)

## ビルドキャッシュを削除
clean:
	arduino-cli cache clean
