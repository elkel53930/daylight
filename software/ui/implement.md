# Raspberry Pi UI Server 実装仕様書

## 概要

Raspberry Pi Compute Module 4上で動作するUIサーバーを実装する。

本サーバーは以下のハードウェアを管理する。

- SSD1331 96×64 フルカラーOLEDディスプレイ
- タクトスイッチ(L/R)
- PWMブザー

これらのデバイスを直接利用するアプリケーションは存在せず、すべてUnix Domain Socket経由でサーバーへアクセスする。

サーバーはsystemdサービスとして常駐し、自動起動・自動再起動される。

---

# 動作環境

- Raspberry Pi Compute Module 4
- Raspberry Pi OS Bookworm (Headless)
- Python 3.11以上
- lgpio
- Pillow
- luma.core
- luma.oled
- msgpack

---

# ディレクトリ構成

```
project/
├── ui_server.py
├── ui_client.py
├── ui_server.service
├── README.md
│
├── resources/
│   └── splash.png
│
├── examples/
│   ├── display_image.py
│   ├── button_test.py
│   └── melody_test.py
│
└── tests/
    └── test_protocol.py
```

---

# ハードウェア

## OLED

- SSD1331
- 96×64
- SPI接続
- luma.coreを使用
- `digital_clock.py`を参考に実装すること

描画はPIL.Imageを受け取り表示する。

---

## ボタン

GPIO

|GPIO|機能|
|---|---|
|19|LEFT|
|26|RIGHT|

押下時GNDへ接続される。

内部Pull-upを利用すること。

デバウンス時間

```
20ms
```

長押し判定

```
1000ms
```

---

## ブザー

GPIO14

ハードウェアPWMを利用すること。

lgpioで実装すること。

---

# アーキテクチャ

サーバー内部は最低限以下のクラスへ責務分離すること。

```
DisplayManager
```

責務

- OLED初期化
- splash表示
- image表示
- clear

---

```
ButtonManager
```

責務

- GPIO監視
- デバウンス
- 長押し判定

---

```
BuzzerManager
```

責務

- PWM初期化
- メロディ再生

---

```
ClientManager
```

責務

- Socket通信
- MessagePack
- 優先度制御

---

```
UIServer
```

責務

- 全体管理
- メインループ

---

# Socket

Unix Domain Socketを利用する。

```
/run/ui_server.sock
```

を使用すること。

---

# 通信

MessagePackを使用する。

JSONは禁止。

---

# クライアント接続

クライアントは

```
connect(priority)
```

で接続する。

優先度

```
0
```

が最優先。

数値が小さいほど優先される。

---

## 優先度制御

接続中のクライアントより高い優先度の接続が来た場合

サーバーは既存クライアントへ

```
PREEMPTED
```

を送信する。

その後Socketを閉じる。

既存クライアントは切断を検知して終了すること。

---

# クライアント数

常時1クライアントのみ。

クライアント切断後、新しい接続を待つ。

---

# 通信プロトコル

MessagePackの辞書形式とする。

## 接続

```
{
    "cmd":"connect",
    "priority":3
}
```

---

## Display

```
{
    "cmd":"display",
    "width":96,
    "height":64,
    "image":bytes
}
```

imageはRGB888の生バイト列。

PNG等への圧縮は禁止。

サイズ

```
96 × 64 × 3
```

固定。

---

## Clear

```
{
    "cmd":"clear"
}
```

---

## Buttons

```
{
    "cmd":"buttons"
}
```

戻り値

```
{
    "left":"released",
    "right":"pressed"
}
```

状態は以下のみ。

```
released
pressed
long_pressed
```

---

## Melody

```
{
    "cmd":"play",
    "melody":"ccddeeff"
}
```

新しいplay要求が来たら現在の演奏を中断し、新しい演奏を開始する。

---

# OLED仕様

API

```
display(image)
```

imageは

```
PIL.Image(mode="RGB")
```

のみ受け付ける。

画像サイズは

```
96×64
```

固定。

それ以外は

```
ValueError
```

を送出すること。

サーバー側でRGB565へ変換して描画する。

描画要求は即座に画面へ反映すること。

---

# Clear

```
clear()
```

画面を黒で塗りつぶす。

---

# ボタン仕様

状態は

```
released
pressed
long_pressed
```

のみ。

イベントキューは不要。

問い合わせ時点での現在状態のみ返す。

---

# ブザー仕様

API

```
play(melody)
```

メロディは1文字ずつ解釈する。

```
c d e f g a b
C D E F G A B
```

以外は休符とする。

各音長

```
150ms
```

Rust実装と同じ仕様にする。

周波数

|文字|Hz|
|---|---:|
|c|524|
|d|588|
|e|660|
|f|698|
|g|784|
|a|880|
|b|988|
|C|1048|
|D|1176|
|E|1320|
|F|1396|
|G|1568|
|A|1760|
|B|1976|

---

# 起動時

起動時は

```
resources/splash.png
```

を表示する。

存在しない場合は黒画面。

---

# 終了処理

SIGTERM受信時

- PWM停止
- OLED消灯（黒画面）
- Socket削除
- GPIO解放

を実施する。

---

# エラー処理

クライアント異常終了

- サーバーは継続動作

SPIエラー

- ログ出力
- サーバー継続

PWMエラー

- ログ出力
- サーバー継続

GPIOエラー

- ログ出力
- サーバー継続

例外でサーバーを停止させてはならない。

---

# ログ

loggingを利用する。

標準出力へ出力する。

systemd journalのみ利用する。

ファイルログは禁止。

---

# systemd

サービス名

```
ui_server.service
```

起動時自動起動。

異常終了時

```
Restart=always
```

とする。

---

# ui_client.py API

```python
client = UIClient()

client.connect(priority)

client.disconnect()

client.display(image)

client.clear()

client.play("ccddeeff")

buttons = client.get_buttons()
```

ボタン取得例

```python
{
    "left":"released",
    "right":"long_pressed"
}
```

---

# examples

## display_image.py

- PNG読込
- PIL.Imageへ変換
- display()

---

## button_test.py

100ms毎にボタン状態表示

---

## melody_test.py

サンプルメロディを再生

---

# tests

最低限以下を作成すること。

- MessagePack通信テスト
- 優先度制御
- PREEMPTED送信
- ボタン取得
- OLED描画
- ブザー再生
- クライアント切断
- サーバー再接続
- SIGTERM処理

---

# README.md

READMEには最低限以下を記載すること。

- 概要
- 必要パッケージ
- インストール方法
- systemdセットアップ
- Socket仕様
- API一覧
- サンプル実行方法
- トラブルシューティング

---

# 実装方針

- 可読性を優先すること。
- 型ヒントを付与すること。
- docstringを記述すること。
- クラスごとに責務を明確に分離すること。
- スレッド安全性を考慮すること。
- 例外は握りつぶさずログへ記録すること。
- 通信はMessagePackのみを使用すること。
- 将来の機能追加（LED、ロータリエンコーダ、センサ等）を考慮し、コマンドディスパッチ方式で実装すること。
- AIエージェントは、保守性・拡張性を重視した実装を行うこと。