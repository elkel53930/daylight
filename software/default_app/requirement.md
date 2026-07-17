# Raspberry Pi Robot Default UI Application 実装仕様書

## 1. 概要

本プロジェクトは、手のひらサイズの小型ロボット上で動作する、常駐型のデフォルトユーザーインターフェースアプリケーションを実装する。

ロボットには以下のユーザーインターフェースがある。

- 96×64 SSD1331フルカラーOLEDディスプレイ
- L/Rの2つのタクトスイッチ
- PWMブザー

これらのハードウェアは、別プロジェクトの常駐プロセスである `ui_server` が管理する。

本アプリケーションは、`ui_client.py` を利用して `ui_server` に接続し、以下の機能を提供する。

- ロボットの状態表示
- バッテリー電圧表示
- CPU温度表示
- CPU周波数表示
- Wi-FiのIPアドレス表示
- 登録されたアプリケーションの起動
- システム再起動
- システムシャットダウン
- バッテリー低電圧警告
- L/Rボタンによるメニュー操作

本アプリケーションはsystemdサービスとして常駐する。

---

# 2. 動作環境

- Raspberry Pi 4
- Raspberry Pi OS Bookworm
- Headless環境
- Python 3
- 既存の `ui_server`
- 既存の `ui_client.py`

必要なPythonライブラリは、既存のUIライブラリ側の仕様に従う。

追加で必要なライブラリ：

- PyYAML

MCP3221のI2C通信にはPython標準ライブラリまたは適切なI2Cライブラリを使用する。

---

# 3. 既存UIサーバーとの関係

システム全体は以下の構成とする。

```text
┌──────────────────────────────┐
│      Default UI Application  │
│                              │
│  Status                      │
│  Application Menu            │
│  System Menu                 │
└──────────────┬───────────────┘
               │
               │ ui_client.py
               │ priority=100
               ▼
┌──────────────────────────────┐
│          ui_server           │
│                              │
│  SSD1331 OLED                │
│  L/R Buttons                 │
│  PWM Buzzer                  │
└──────────────┬───────────────┘
               │
               │
        ┌──────┴──────┐
        │             │
   OLED Display   Hardware
                  Devices
```

他のアプリケーションが起動すると、より高い優先度で `ui_server` に接続する。

```text
Default UI       priority=100
Other App        priority=10
```

優先度の数値が小さいほど優先度が高い。

他のアプリケーションが接続すると、Default UIは `PREEMPTED` 通知を受けてUIを解放する。

他のアプリケーションが終了すると、Default UIはUIサーバーへ再接続して処理を再開する。

---

# 4. UI優先度

Default UIの優先度：

```text
100
```

他のアプリケーションは、通常これより高い優先度を使用する。

例：

```text
Default UI: 100
Robot Control: 10
Motor Test: 20
Diagnostic: 30
```

---

# 5. ハードウェア

## 5.1 OLED

OLEDの制御は直接行わない。

既存の `ui_client.py` を利用する。

画面サイズ：

```text
96×64
```

画像形式：

```text
PIL.Image(mode="RGB")
```

---

## 5.2 ボタン

|ボタン|機能|
|---|---|
|L|次のメニューへ移動|
|R|現在のメニューを決定|

### Lボタン

Lを短押しすると次のメニューへ移動する。

最後のメニューでLを押した場合、先頭のメニューへ戻る。

```text
A → B → C → A
```

### Rボタン

Rを短押しすると、現在選択されているメニューを決定する。

### 長押し

現時点では使用しない。

---

# 6. メイン画面

メイン画面は以下の構成とする。

```text
┌────────────────────────────────┐
│ IP: 192.168.1.10               │
│                                │
│ BAT: 7.42V                     │
│                                │
│ > Applications                 │
│   System                       │
└────────────────────────────────┘
```

ただし、OLEDは96×64ピクセルであるため、実際のフォントサイズに応じて表示レイアウトを調整すること。

---

## 6.1 IPアドレス

Wi-FiインターフェースのIPv4アドレスを表示する。

対象インターフェース：

```text
wlan0
```

例：

```text
IP: 192.168.1.10
```

IPアドレスは常時表示する。

IPアドレスを取得できない場合：

```text
IP: N/A
```

と表示する。

---

## 6.2 ステータス表示

以下の情報を一定間隔で切り替えて表示する。

- バッテリー電圧
- CPU温度
- CPU周波数

表示例：

```text
BAT: 7.42V
```

```text
CPU: 48.7C
```

```text
FREQ: 1500MHz
```

表示切り替え間隔：

```text
2秒
```

表示順：

```text
Battery
↓
CPU Temperature
↓
CPU Frequency
↓
Battery
```

---

# 7. CPU温度

CPU温度は `vcgencmd` を利用して取得する。

実行コマンド：

```bash
vcgencmd measure_temp
```

典型的な出力：

```text
temp=48.7'C
```

これを解析して温度を取得する。

表示例：

```text
CPU: 48.7C
```

取得に失敗した場合：

```text
CPU: N/A
```

と表示する。

例外でDefault UIを終了させてはならない。

---

# 8. CPU周波数

以下のファイルを読み取る。

```text
/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_cur_freq
```

値はHzである。

例：

```text
1500000
```

表示時にMHzへ変換する。

```text
1500000 Hz
→
1500 MHz
```

表示例：

```text
FREQ: 1500MHz
```

取得に失敗した場合：

```text
FREQ: N/A
```

と表示する。

---

# 9. バッテリー電圧

## 9.1 ADC

MCP3221を使用する。

I2C接続：

|信号|GPIO|
|---|---|
|SDA|GPIO2|
|SCL|GPIO3|

I2Cバス：

```text
/dev/i2c-1
```

MCP3221のI2Cアドレスは、実装時にハードウェアのデータシートおよび実機配線に基づいて設定すること。

アドレスを推測してハードコードする場合は、設定値として変更可能にすること。

---

## 9.2 ADC変換

MCP3221は12-bit ADCとして扱う。

ADCの最大値：

```text
4095
```

VDD：

```text
3.3V
```

ADC入力電圧：

```text
adc_voltage = raw_value / 4095 * 3.3
```

バッテリー電圧：

```text
battery_voltage = adc_voltage * 11
```

したがって：

```text
battery_voltage =
    raw_value / 4095 * 3.3 * 11
```

---

## 9.3 監視周期

バッテリー電圧はバックグラウンドで定期的に取得する。

取得間隔：

```text
2秒
```

UI表示の切り替え処理とは分離すること。

---

# 10. バッテリー低電圧警告

危険電圧：

```text
6.5V未満
```

バッテリー電圧が6.5V未満になった場合：

1. 画面に低電圧警告を表示
2. ブザーで警告音を鳴らす

表示例：

```text
BATTERY LOW
```

または画面レイアウトに合わせて：

```text
LOW BATTERY
```

---

## 10.1 ブザー警告

既存の `ui_client.play()` を使用する。

警告音は短いメロディを使用する。

例：

```text
"cc"
```

または同等の短い警告音。

低電圧状態が継続している場合、ブザーを連続的に鳴らし続けてはならない。

警告音は一定間隔で再生する。

推奨：

```text
警告音
↓
数秒待機
↓
警告音
```

警告間隔は設定値として変更可能にする。

---

## 10.2 ヒステリシス

バッテリー電圧が測定誤差によって6.5V付近を上下する場合、警告状態が頻繁に切り替わらないようにする。

推奨：

```text
低電圧警告開始:
    < 6.5V

低電圧警告解除:
    >= 6.7V
```

このヒステリシス値は設定可能にする。

---

# 11. メニュー構造

メインメニュー：

```text
Applications
System
```

必要に応じて将来拡張可能な設計にする。

---

## 11.1 Applications

YAMLで定義されたアプリケーション一覧を表示する。

例：

```text
Applications

> Maze
  Motor Test
  Diagnostics
```

L：

```text
次のアプリケーション
```

最後の項目でL：

```text
先頭へ戻る
```

R：

```text
選択したアプリケーションを起動
```

---

## 11.2 System

システムメニュー：

```text
System

> Reboot
  Shutdown
```

L：

```text
次の項目
```

R：

```text
選択
```

---

# 12. アプリケーション設定

アプリケーション一覧はYAMLファイルから読み込む。

配置場所：

```text
/etc/robot-ui/applications.yaml
```

---

## 12.1 YAML形式

例：

```yaml
applications:
  - name: Maze
    command:
      - python3
      - /opt/robot/apps/maze.py
    priority: 10

  - name: Motor Test
    command:
      - python3
      - /opt/robot/apps/motor_test.py
    priority: 20
```

---

## 12.2 設定項目

### name

UI上に表示するアプリケーション名。

必須。

---

### command

アプリケーション起動コマンド。

安全性のため、可能であれば文字列をシェル経由で実行せず、引数リストとして実行する。

推奨：

```yaml
command:
  - python3
  - /opt/robot/apps/example.py
```

`subprocess.Popen()`では以下のように実行する。

```python
subprocess.Popen(command, shell=False)
```

---

### priority

アプリケーションがUIサーバーへ接続する際の優先度。

例：

```yaml
priority: 10
```

この値は、アプリケーション自身が利用する値として保持する。

Default UIの優先度は100。

---

# 13. アプリケーション起動

アプリケーションは `subprocess.Popen()` で起動する。

例：

```python
process = subprocess.Popen(command, shell=False)
```

起動後、Default UIは以下の処理を行う。

1. 子プロセスを起動
2. UIサーバーとの接続を解放
3. 子プロセスの終了を待つ
4. 子プロセス終了後、UIサーバーへ再接続
5. Default UIを再開
6. メイン画面へ戻る

---

# 14. PREEMPTED処理

Default UIがui_serverから `PREEMPTED` を受信した場合：

1. UIクライアント接続を閉じる
2. 現在のUI処理を停止
3. Default UIの処理を一時停止
4. ui_serverからの接続可能状態を待つ
5. 再接続する
6. メイン画面を再描画する

Default UIは `PREEMPTED` を受信したことをエラー扱いして終了してはならない。

---

# 15. アプリケーション終了後

起動したアプリケーションが終了したら、Default UIはUIサーバーへ再接続する。

再接続後：

```text
Main Screen
```

へ戻る。

メニュー選択状態は先頭へ戻す。

---

# 16. 再接続

以下の場合に再接続を行う。

- PREEMPTED後
- 子アプリケーション終了後
- ui_server再起動後
- Socket切断後

再接続はリトライする。

推奨：

```text
1秒間隔
```

ただし、無限に高速リトライしてCPUを消費してはならない。

---

# 17. Systemメニュー

## 17.1 Reboot

Rebootを選択すると確認画面を表示する。

例：

```text
Reboot?

L: No
R: Yes
```

L：

```text
キャンセル
```

R：

```text
再起動実行
```

実行時：

```bash
sudo systemctl reboot
```

または適切なsystemd APIを利用する。

---

## 17.2 Shutdown

Shutdownを選択すると確認画面を表示する。

例：

```text
Shutdown?

L: No
R: Yes
```

L：

```text
キャンセル
```

R：

```text
シャットダウン実行
```

実行時：

```bash
sudo systemctl poweroff
```

または適切なsystemd APIを利用する。

---

# 18. 権限

CPU温度取得のため：

```bash
vcgencmd measure_temp
```

を実行する。

CPU周波数ファイルを読み取る。

システム再起動・シャットダウンを実行するため、必要な権限をsystemdサービス側で適切に設定する。

可能な限り、アプリケーション全体をrootで実行するのではなく、必要な操作のみsudoersまたはsystemd経由で許可すること。

---

# 19. 起動時動作

Default UI起動時：

1. 設定ファイルを読み込む
2. アプリケーション一覧を構築
3. システム情報取得処理を開始
4. バッテリー監視処理を開始
5. ui_serverへpriority=100で接続
6. メイン画面を表示
7. ボタン入力を処理

---

# 20. 画面更新

画面更新処理とハードウェア監視処理は分離する。

推奨構成：

```text
┌────────────────────────┐
│       Main Loop        │
│                        │
│  UI State              │
│  Button Polling        │
│  Rendering             │
└────────────┬───────────┘
             │
             ├───────────────┐
             │               │
             ▼               ▼
     Status Monitor    Battery Monitor
```

CPU温度、CPU周波数、IPアドレスの取得でUI処理をブロックしないこと。

---

# 21. 推奨クラス構成

ファイル数は過度に増やさず、責務を明確に分離すること。

推奨：

```text
DefaultUI
```

アプリケーション全体を統括する。

---

```text
MenuManager
```

責務：

- メニュー項目管理
- 現在の選択項目
- Lによる移動
- Rによる決定

---

```text
ApplicationManager
```

責務：

- YAML読み込み
- アプリケーション一覧管理
- subprocess起動
- 子プロセス終了待機

---

```text
SystemInfo
```

責務：

- Wi-Fi IPアドレス取得
- CPU温度取得
- CPU周波数取得

---

```text
BatteryMonitor
```

責務：

- MCP3221読み取り
- ADC値から電圧への変換
- 低電圧判定
- 警告状態管理

---

```text
UIRenderer
```

責務：

- PIL.Image生成
- メイン画面描画
- メニュー描画
- 警告表示
- ui_client.display()呼び出し

---

```text
SystemController
```

責務：

- Reboot
- Shutdown

---

```text
DefaultUI
```

責務：

- 各クラス統合
- メインループ
- ui_serverへの接続
- PREEMPTED処理
- 再接続

---

# 22. UI状態

最低限、以下の状態を持つ。

```text
MAIN
APPLICATION_MENU
SYSTEM_MENU
CONFIRM_REBOOT
CONFIRM_SHUTDOWN
LOW_BATTERY
```

ただし、低電圧警告は独立した画面状態ではなく、通常画面にオーバーレイする設計でもよい。

---

# 23. ボタンポーリング

既存の `ui_client.get_buttons()` を使用する。

ポーリング間隔：

```text
100ms程度
```

ボタン状態が以下の場合に処理する。

```text
pressed
```

`long_pressed` は現時点では無視する。

同じ押下を複数回処理しないようにすること。

例：

```text
pressed
↓
処理
↓
released待ち
```

---

# 24. 描画方式

Pillowを利用して画面を生成する。

画像サイズ：

```text
96×64
```

画像モード：

```text
RGB
```

生成した画像を既存の：

```python
ui_client.display(image)
```

へ渡す。

---

# 25. フォント

フォントは設定可能にする。

デフォルトフォントはシステムに存在するTrueTypeフォントを使用する。

96×64という制約があるため、以下を考慮すること。

- 文字サイズ
- 1行あたりの文字数
- IPアドレスの表示
- アプリケーション名の長さ
- メニュー項目の長さ

長いアプリケーション名は以下のいずれかで処理する。

推奨：

```text
自動的に省略
```

例：

```text
Very Long Applic...
```

---

# 26. YAMLエラー処理

設定ファイルが存在しない場合：

```text
アプリケーション一覧を空として扱う
```

設定ファイルの構文エラー：

- エラーをsystemd journalへ出力
- Default UI自体は起動継続
- Applicationsメニューを空として扱う

不正なアプリケーションエントリ：

- 該当エントリをスキップ
- ログへ警告を出力

---

# 27. エラー処理

以下のエラーが発生しても、Default UI全体を停止させない。

- MCP3221読み取り失敗
- IPアドレス取得失敗
- CPU温度取得失敗
- CPU周波数取得失敗
- YAML読み込み失敗
- ui_server接続失敗
- アプリケーション起動失敗

可能な限り：

```text
N/A
```

などの代替表示を行う。

すべての例外は適切にログへ記録する。

---

# 28. ログ

Pythonの `logging` を使用する。

ログは標準出力または標準エラー出力へ出力する。

systemd journalで確認可能にする。

ファイルログは作成しない。

ログレベル：

- DEBUG
- INFO
- WARNING
- ERROR

通常運用時はINFO以上を出力する。

---

# 29. systemdサービス

サービス名：

```text
default-ui.service
```

例：

```ini
[Unit]
Description=Robot Default UI
Requires=ui-server.service
After=ui-server.service

[Service]
Type=simple
ExecStart=/usr/bin/python3 /opt/robot-ui/default_ui.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

実際のパスは環境に合わせて設定する。

---

# 30. systemdサービスの動作

Default UIが異常終了した場合：

```text
Restart=always
```

により自動再起動する。

ui_serverが再起動した場合、Default UIはSocket切断を検知し、再接続する。

---

# 31. 推奨ディレクトリ構成

```text
robot-default-ui/
├── default_ui.py
├── menu.py
├── application_manager.py
├── system_info.py
├── battery.py
├── renderer.py
├── system_controller.py
│
├── config/
│   └── applications.yaml.example
│
├── tests/
│   ├── test_menu.py
│   ├── test_battery.py
│   ├── test_system_info.py
│   ├── test_application_manager.py
│   └── test_renderer.py
│
├── default-ui.service
└── README.md
```

実際の `/etc/robot-ui/applications.yaml` は、インストール時に作成する。

---

# 32. テスト

最低限、以下のテストを作成する。

## MenuManager

- Lで次の項目へ移動
- 最後の項目から先頭へ循環
- 空のメニュー
- 1項目だけのメニュー

---

## BatteryMonitor

- ADC値0
- ADC値4095
- 通常電圧
- 6.5V未満の低電圧
- 低電圧解除ヒステリシス
- I2C読み取り失敗

---

## SystemInfo

- CPU温度の解析
- `vcgencmd`失敗
- CPU周波数のHz→MHz変換
- ファイル読み取り失敗
- Wi-Fi IPアドレス取得
- IPアドレス未取得

---

## ApplicationManager

- YAML読み込み
- 正常なアプリケーション定義
- YAML構文エラー
- 不正なエントリ
- subprocess起動
- subprocess終了

---

## Renderer

- 96×64の画像が生成される
- RGBモードである
- メニュー項目が描画される
- バッテリー情報が描画される
- CPU情報が描画される
- IPアドレスが描画される
-低電圧警告が描画される

---

# 33. README.md

READMEには以下を記載する。

## 概要

Default UIの役割。

---

## 必要環境

- Raspberry Pi 4
- Raspberry Pi OS Bookworm
- ui_server
- Python 3

---

## インストール

例：

```bash
git clone ...
cd robot-default-ui
pip install -r requirements.txt
```

---

## 設定

```bash
sudo mkdir -p /etc/robot-ui
sudo cp config/applications.yaml.example \
    /etc/robot-ui/applications.yaml
```

---

## アプリケーション追加

YAMLを編集するだけで追加可能にする。

```yaml
applications:
  - name: Example
    command:
      - python3
      - /opt/robot/apps/example.py
    priority: 10
```

---

## systemd登録

```bash
sudo cp default-ui.service \
    /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable default-ui.service
sudo systemctl start default-ui.service
```

---

## ログ確認

```bash
journalctl -u default-ui.service -f
```

---

## トラブルシューティング

以下を含める。

- ui_serverに接続できない
- MCP3221が読めない
- IPアドレスが表示されない
- CPU温度が取得できない
- アプリケーションが起動しない
- PREEMPTED後に再接続できない

---

# 34. 実装上の重要事項

AIエージェントは以下を必ず守ること。

1. 既存の `ui_client.py` のAPI仕様に従うこと。
2. OLEDへ直接アクセスしないこと。
3. GPIOへ直接アクセスしないこと。
4. ブザーへ直接アクセスしないこと。
5. すべてのUIハードウェア操作はui_server経由で行うこと。
6. Default UIの優先度は100とすること。
7. 他アプリケーションのUIを妨害しないこと。
8. `PREEMPTED` を正常な状態遷移として扱うこと。
9. UIサーバー切断時に自動再接続すること。
10. アプリケーション追加時にPythonコードの変更を不要とすること。
11. YAMLの不正な項目によってDefault UI全体を停止させないこと。
12. バッテリー電圧監視と画面描画を分離すること。
13. バッテリー低電圧時にブザーを連続再生しないこと。
14. 例外を握りつぶさず、systemd journalへ記録すること。
15. テストコードを作成すること。
16. README.mdを作成すること。
17. 型ヒントとdocstringを使用すること。
18. 将来のメニュー追加、センサー追加、アプリケーション追加を考慮した設計にすること。

---

# 35. 完成条件

以下を満たした場合、実装完了とする。

- Default UIがsystemdで自動起動する
- ui_serverへpriority=100で接続する
- OLEDにIPアドレスが表示される
- バッテリー電圧が表示される
- CPU温度が表示される
- CPU周波数が表示される
- ステータス情報が一定間隔で切り替わる
- Lでメニューが循環する
- Rでメニューを決定できる
- YAMLからアプリケーション一覧を読み込める
- アプリケーションを起動できる
- アプリケーション起動中はUIを解放できる
- アプリケーション終了後にUIを再取得できる
- Rebootメニューが動作する
- Shutdownメニューが動作する
- 確認画面でL/Rを使ってYes/Noを選択できる
- バッテリー電圧6.5V未満で警告する
- 低電圧時にブザーが鳴る
- 低電圧時に画面警告を表示する
- ui_server再起動後に再接続できる
- Default UI異常終了時にsystemdが自動再起動する
- テストが実行できる
- README.mdにセットアップ手順が記載されている