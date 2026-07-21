# micromouse

Daylight のマイクロマウス自律走行アプリケーション。ラズパイ CM4 上で動作し、
シリアル経由で mob (ESP32-S3) に走行コマンドを送って 16×16 迷路の探索・
最短経路走行を行う。

参考実装は前身機 [robosweep_twilight](https://github.com/elkel53930/robosweep_twilight)。
移植方針と差分は末尾の「Twilight からの移植」を参照。

---

## アーキテクチャ

```
micromouse_app.py            エントリポイント (UI統合・引数処理)
    │
state_machine.py             ミッション状態機械 (探索→帰還→計画→最短走行)
    │
    ├── explorer.py          足立法探索 (次に進む方位の決定)
    ├── path_planner.py      最短経路計算と走行命令列への変換
    ├── maze.py              迷路データ構造 (壁3状態・訪問・距離マップ)
    ├── wall_detector.py     センサ値→壁有無の判定 / SENパース
    └── cell_runner.py       セル単位走行 (1セル直進・旋回)
            │
        mobile_base.py       mobシリアルドライバ (FWD/STOP/TURN/SEN...)
     or simulator.py         仮想mob (ハードウェア無しでの検証用)
            │
        /dev/ttyUSB0         mob (ESP32-S3) 3Mbps
```

依存方向は必ず「ハードウェア → センサ抽象 → 迷路アルゴリズム」。
`maze.py` / `explorer.py` / `path_planner.py` はハードウェアに一切依存せず、
ユニットテストのみで検証できる。`simulator.py` は `mobile_base.py` と同じ
インターフェースを持ち、状態機械以上の層はどちらが下にいるかを知らない。

### 座標系

- **迷路座標系**: セル単位 `(x, y)`。スタート `(0, 0)` が左下、x は東へ、
  y は北へ増える。方位は絶対方位 `NORTH/EAST/SOUTH/WEST`。
- **連続座標系**: mob 側のオドメトリ (走行距離 mm・姿勢角 rad)。
  mob の `FWD/STOP/TURN` コマンドが消費する。

変換は `cell_runner.py` だけが行う(迷路座標系の「1セル進む/右を向く」を
連続座標系の「FWD 90mm ×2 / TURN -π/2」に落とす)。逆方向の変換
(連続→迷路)は行わない: セル位置はコマンド完了 (DONE) の積み上げで管理し、
オドメトリは mob 内部の制御にのみ使う。

### 探索時のセルサイクル

Twilight で実績のある「セル境界を判断点とする」方式を踏襲する:

```
  [セル境界 = 判断点]  ← センサはここで進入先セルの壁を読む
       │ SEN → 壁判定 → 迷路更新 → 訪問記録 → ゴール判定 → 次方位決定
       │
  直進: FWD 90mm + FWD 90mm       (境界 → 次の境界)
  旋回: STOP 90mm → TURN ±π/2 → FWD 90mm
                     (セル中心で旋回して次の境界へ)
  ゴール/中断: STOP 90mm          (セル中心で停止)
```

探索終了時は必ずセル中心で停止するため、最短走行はセル中心間の
「直進 n セル (STOP n×180mm) + その場旋回」の繰り返しで実行する。

---

## 状態機械

```
IDLE ──(Rボタン)──▶ CALIBRATION ──▶ MICROMOUSE_START ──▶ EXPLORATION
                                                             │
                     ┌───────────────────────────────────────┘
                     ▼ (ゴール領域のセルに進入し中心で停止)
                GOAL_REACHED ──▶ RETURN_TO_START ──▶ PATH_PLANNING
                                                          │
                     ┌────────────────────────────────────┘
                     ▼ (既知壁のみで経路が引けたら)
                 SPEED_RUN ──(ゴール中心で停止)──▶ FINISHED ──▶ IDLE

任意の状態 ──(異常検出)──▶ ERROR          ──(ボタン)──▶ IDLE
任意の状態 ──(Lボタン)──▶ EMERGENCY_STOP  ──(ボタン)──▶ IDLE
```

遷移条件の定義(曖昧さ回避のため明文化):

| 遷移 | 条件 |
|---|---|
| 探索開始 | `CALIBRATION` 完了(GCAL/RDST/RANG/WALL,1 全て DONE)後、最初の FWD 送信をもって探索開始とする |
| ゴール到達 | 判断点で自セルが `GoalRegion` に含まれる、と判定した時 |
| ゴール後の停止 | ゴール判定した判断点から `STOP 90mm` でそのセルの中心に停止する |
| 最短経路計算 | スタートへ帰還完了(スタートセル中心で停止)後。**未知壁=壁扱い**で BFS |
| 最短走行開始 | 経路が存在し、ユーザーが R ボタンで承認した時(`--autostart` 時は即時) |
| 異常時 | QSTP → MOT,0,0 でモータ停止してから ERROR/EMERGENCY_STOP へ遷移 |

`RETURN_TO_START` はゴール地点の壁情報だけでは最短経路の安全性を保証でき
ないため必須とした(帰還も足立法で行うので追加の壁情報も収集できる)。
`PATH_PLANNING` で未知壁=壁として経路が引けない場合は理論上あり得ない
(帰還時に通った経路が必ず既知)が、防御的に ERROR とする。

### 安全動作

以下を検出したら走行より優先して QSTP → MOT,0,0 → ERROR:

- SEN 取得の連続失敗(リトライ含め5回)
- 走行コマンドの DONE タイムアウト
- バッテリー電圧低下(既定 6.5 V 未満)
- シリアル例外・予期しない例外(except 節で捕捉し必ず停止)
- L ボタン(または Ctrl+C)による中断 → EMERGENCY_STOP

---

## 使い方

### 前提: mob ファームウェアのバージョン

本アプリは mob の FWD/STOP/TURN/QSTP コマンドと 11 フィールドの SEN 応答
(`software/mob/mob.ino` の現行版)を前提とする。ESP32 に古いファームウェア
(SEN が 7 フィールドの版など)が入っている場合は先に書き込むこと:

```bash
cd software/mob && make upload PORT=/dev/ttyUSB0
```

書き込み後 `hw_test.py sen` で 11 フィールドの応答が返ることを確認する。

### 実機

```bash
software/venv/bin/python3 software/micromouse/micromouse_app.py \
    --port /dev/ttyUSB0
```

UI サーバー稼働時は OLED に状態・現在セル・方位が表示され、
R ボタンで開始 / L ボタン長押しで緊急停止。UI が無い環境では
`--no-ui --autostart` でコンソールのみでも動く。

`default_app` のメニューから起動する場合は `/etc/robot-ui/applications.yaml` に:

```yaml
applications:
  - name: Micromouse
    command:
      - /usr/bin/python3
      - /opt/robot/apps/micromouse/micromouse_app.py
      - --port
      - /dev/ttyUSB0
    priority: 20
```

### シミュレーション(ハードウェア不要)

```bash
software/venv/bin/python3 software/micromouse/micromouse_app.py \
    --sim software/micromouse/maze_files/AllJapan_002_1981_classic___16x16.txt \
    --no-ui --autostart
```

仮想迷路上で探索→帰還→最短走行の全シーケンスを実行し、ASCII 迷路表示と
走行ログを出力する。アルゴリズム変更時はまずこれで検証すること。

### 実機の段階的検証 (hw_test.py)

モータを回す前に必ずこの順で確認する:

```bash
V=software/venv/bin/python3 ; M=software/micromouse/hw_test.py
$V $M sen                 # センサ値の連続表示(モータは回らない)
$V $M walls               # 壁判定の連続表示(しきい値確認)
$V $M gcal                # ジャイロキャリブレーション
$V $M fwd                 # 1セル(180mm)直進
$V $M turn left           # 90度左旋回
$V $M turn right          # 90度右旋回
$V $M turn back           # 180度旋回
$V $M cycle               # 探索1サイクル(半セル→判断→半セル)
```

### 固定走行パターンテスト (pattern_test.py)

迷路探索を介さず、決め打ちの動作(右旋回→左旋回→180度旋回、その場
旋回のみで前進なし)を実行するテスト。直進側は別問題を抱えているため
(2026-07-22時点)、まず旋回制御(PID)のオーバーシュート・振動の
チューニングに絞って見るためのパターンにしてある。hw_test.py の
単発 turn コマンドと違い、連続旋回時の誤差累積を見られる。
過去には前進を含む閉路パターン(スタート位置に戻る)だった。復元する
場合は git 履歴の `PATTERN` 定義を参照。

```bash
software/venv/bin/python3 software/micromouse/pattern_test.py
```

`--speed`/`--accel` で速度プロファイルを変更可能(既定は config の
explore 系の値)。`--no-ui` でコンソールモード(ui_server 不要)。
default_app の Applications メニューにも `Pattern Test` として登録済み
(`/etc/robot-ui/applications.yaml`、登録例は
`software/default_app/config/applications.yaml.example` 参照)。

実行のたびに `logs/pattern_YYYYmmdd_HHMMSS.jsonl` へ mob からの生シリアル
行(SEN・DONE に加えて `updateForward`/`updateStop`/`updateTurn`/
`updateQstp` が20Hzで送る `#V,cmd,vr,vl,ur,ul,gz,ang,rem` テレメトリ含む)
をそのまま記録する(`mobile_base.MobileBase` の `raw_log_fn` フック経由。
これが無いと `#V,...` 行は `_wait_for()` の一時バッファでしか保持されず
DONE 受信時に捨てられてしまう)。各行に走行中のステップ番号・区間
(`Motion` の repr)を付与しているので、旋回中のオーバーシュート等は
該当ステップの `#V` 行を time 順に追って `ang`(角度)や `gz`(角速度)の
推移を見れば確認できる。

### 設定

`config/micromouse.yaml.example` を `config/micromouse.yaml` にコピーして編集
(無ければ既定値で動く)。迷路サイズ・セル寸法・ゴール領域・速度・
壁しきい値・バッテリー下限などが変更できる。`--config` で任意パスも指定可。

### テスト

```bash
software/venv/bin/python3 -m unittest discover -s software/micromouse/tests -q
```

3層構成: ユニットテスト(迷路・探索・経路)、シミュレーションテスト
(仮想迷路でエンドツーエンド)、実機テスト(hw_test.py、手動)。

### 走行ログ

実行ごとに `logs/run_YYYYmmdd_HHMMSS.jsonl` に1判断点1行で記録:
timestamp / state / セル座標 / 方位 / センサ生値 / 壁判定 / 次アクション。
終了時に探索済み迷路を `logs/maze_*.json` に保存する(再走行・解析用)。

---

## Twilight からの移植

### 機能マップ

| Twilight機能 | 実装ファイル (twilight) | ハード依存 | Daylight対応 | 移植方針 |
|---|---|---|---|---|
| 足立法・距離マップ・迷路表現 | `software/search/micromouse_algorithms.py` | なし | 同等品なし | アルゴリズム再利用。壁を bool×2面(known/observed)から3状態 `WallState` に再設計し、Maze/Explorer/PathPlanner を分離 |
| 迷路ファイル読み込み | 同上 (`read_maze_from_text_file`) | なし | 同等品なし | シミュレータ用に移植 (`simulator.py`) |
| mobシリアルドライバ | `software/mob/mobile_base*.py` | プロトコル依存 | プロトコルほぼ同一 | 同期版として再実装。スレッド化はせず、DONE待ちループに中断コールバックを挿す方式に簡素化 |
| 探索走行ループ(セル境界判断・90+90mm走行) | `software/solve_maze_threaded.py` | 機体寸法依存 | 機体構成類似 | 構造を再設計して移植。ボール/アーム処理は削除し、状態機械 (`state_machine.py`) として明確化 |
| 壁しきい値判定 | `solve_maze_threaded.py` (`detect_walls`) | センサ個体依存 | センサ構成が異なる(下記) | ロジック再利用、しきい値は設定ファイル化して実機で校正 |
| 低レベル走行 (FWD/STOP/TURN/速度PID/壁補正) | `software/mob/*.cpp` | ESP32 | **Daylight mobに実装済み** | 移植しない(Daylight既存実装を使用) |
| 最短経路走行 | (存在しない) | — | — | 新規実装 (`path_planner.py` + SPEED_RUN) |
| ボール検出・アーム | `rpi/camera_py`, `arm/` | RoboSweep固有 | 対象外 | 移植しない |

### ハードウェア差分

| 項目 | Twilight | Daylight |
|---|---|---|
| 壁センサ | 左前/右前 + 左横/右横 の4個(外付けADC) | **前1 + 左横/右横 の3個**(内蔵ADC)。SEN の `lf`/`rf` はどちらも前センサ値を返す |
| 前壁判定 | `lf>=50 AND rf>=50` | 前センサ1個なので実質単一しきい値(同式のまま動くが意味が変わる) |
| シリアルポート | `/dev/ttyMOB` (udev alias) | `/dev/ttyUSB0` (FTDI FT231X) |
| 付加機構 | アーム・投擲・カメラ | 吸引ファン・サーボ(マイクロマウスでは未使用) |
| UI | なし(SSH運用) | OLED + 2ボタン + ブザー (`ui_server`) |

### 挙動を変えた箇所と理由

- **壁3状態 (UNKNOWN/OPEN/WALL)**: Twilight は known/observed の bool 2面
  持ちだった。等価だが、指示書の要件(未知壁の明示)に合わせ単一の
  enum に再設計。隣接セルとの整合は `Maze.set_wall()` が常に両面更新する。
- **探索と経路計画の分離**: Twilight の `AdachiExplorer.decide_heading()` は
  壁更新+距離再計算+方位決定+自己位置更新が一体だった。テスト可能性の
  ため、位置(Pose)は状態機械が所有し、Explorer は「迷路+位置→次方位」の
  純粋な決定のみ行う。
- **未知壁の扱い**: 探索時は未知=通行可(楽観)、最短走行の計画時は
  未知=壁(安全)。Twilight は前者のみだった。
- **スレッド化しない**: Twilight のスレッド版はボール検出との並行性のため
  だった。マイクロマウスでは判断点間が全て同期処理で足りるため、
  シリアルは同期ドライバ+中断コールバックに簡素化(保守性優先)。
- **バッテリー監視**: SEN に電圧が載っているため判断点ごとに下限チェックを
  追加(Twilight には無かった安全処理)。

---

## 完了条件との対応

- ソフトウェア項目(Maze/壁整合/位置方向/ゴール判定/探索/最短経路/命令変換/状態機械)
  → `tests/` のユニットテスト + シミュレーションテストで検証
- ハードウェア項目(センサ/エンコーダ/モータ/直進/旋回/セル走行)
  → `hw_test.py` で段階的に実機検証
- 実機項目(未知迷路探索〜最短再走行〜異常停止)
  → `micromouse_app.py` を実迷路で実行して検証
