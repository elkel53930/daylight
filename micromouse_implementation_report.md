# マイクロマウス機能 実装報告

- 日付: 2026-07-17
- 指示書: `implement_micromouse.md`
- 実装先: `software/micromouse/`(新規ディレクトリ。既存コードの変更は最小限)
- 参考実装: [robosweep_twilight](https://github.com/elkel53930/robosweep_twilight)

## 結論(サマリ)

- 迷路探索(足立法)→ ゴール到達 → スタート帰還 → 最短経路計画 → 最短走行の
  全シーケンスを実装し、**シミュレーション上で全日本大会の実迷路2面
  (1981年・1993年)を完走**することを確認した。
- テストは **64件すべてパス**(迷路コアのユニットテスト+仮想迷路での
  エンドツーエンドのシミュレーションテスト)。既存の `default_app` の
  テスト(97件)もパスしており、既存機能は壊していない。
- 実機(CM4)から mob との**シリアル疎通を確認済み**(モータは回していない)。
  ただし実機走行の前にユーザー作業が2点残っている(後述)。

---

## 1. 調査結果 (Phase 1)

### Daylight の構成

| 要素 | 内容 |
|---|---|
| 計算機 | Raspberry Pi CM4(Python アプリ群)+ ESP32-S3 "mob"(Arduino/C++ ファームウェア) |
| 通信 | USB シリアル `/dev/ttyUSB0`(FTDI FT231X)、3 Mbps、CSV 行プロトコル |
| 走行制御 | **mob 側に実装済み**: FWD(台形加速・距離指定)/ STOP(距離指定停止)/ TURN(その場旋回)/ QSTP(クイック停止)/ GCAL / RDST / RANG。1 kHz 制御ループ、車輪速度 PID、角度・角速度フィードバック、壁センサ横補正 |
| センサ | 壁センサ3個(前・左横・右横、LED差分方式)、AS5047 エンコーダ×2、LSM6DSR ジャイロ、バッテリー電圧。`SEN` コマンドでhttps://x.com/i/status/2077768657552523681一括取得(11フィールド) |
| UI | OLED 96×64 + ボタン L/R + ブザー(`ui_server` 経由、Unix ドメインソケット) |
| ラズパイ側 | mob と通信する Python コードは**存在しなかった**(今回新規作成) |

### Twilight の再利用可能資産

| Twilight機能 | 実装ファイル | ハード依存 | Daylight対応 | 移植方針 |
|---|---|---|---|---|
| 足立法・距離マップ・迷路表現 | `search/micromouse_algorithms.py` | なし | 同等品なし | アルゴリズム再利用、構造は再設計 |
| mobシリアルドライバ | `mob/mobile_base*.py` | プロトコル依存 | プロトコルほぼ同一 | 同期版として簡素化して移植 |
| 探索走行ループ(セル境界判断・半セル90mm×2走行) | `solve_maze_threaded.py` | 機体寸法依存 | 機体構成類似 | 構造を状態機械として再設計、ボール/アーム処理は削除 |
| 壁しきい値判定 | `solve_maze_threaded.py` | センサ個体依存 | センサ構成が異なる | ロジック再利用、しきい値は設定ファイル化 |
| 低レベル走行制御 | `mob/*.cpp` | ESP32 | **Daylight mob に実装済み** | 移植不要(既存を使用) |
| 最短経路走行 | (存在しない) | — | — | 新規実装 |

### 主なハードウェア差分

- 壁センサ: Twilight は 4個(左前/右前/左横/右横)、Daylight は **3個**(前/左横/右横)。
  SEN の `lf`/`rf` はどちらも前センサ値を返すため、`front = min(lf, rf)` として
  判定すれば Twilight の「両方しきい値以上で前壁」と等価になる。
- シリアルポート: `/dev/ttyMOB`(udev alias)→ `/dev/ttyUSB0`。
- Daylight には OLED/ボタン UI があるため、SSH 不要で開始・中断できるようにした。

---

## 2. 実装 (Phase 2〜7)

### ファイル構成

```
software/micromouse/
├── README.md            設計・使い方・Twilight移植方針の文書
├── maze.py              迷路データ構造(壁3状態・訪問・距離マップ・ASCII表示)
├── explorer.py          足立法探索(次方位の決定)
├── path_planner.py      最短経路計算+走行命令列への変換
├── wall_detector.py     SENパース+しきい値壁判定
├── mobile_base.py       mobシリアルドライバ(実機)
├── simulator.py         仮想mob(同一インターフェース、衝突検出付き)
├── cell_runner.py       セル単位走行(迷路座標系→連続座標系の変換層)
├── state_machine.py     ミッション状態機械
├── micromouse_app.py    エントリポイント(UI統合・走行ログ)
├── hw_test.py           実機の段階的検証CLI
├── errors.py            共有例外(pyserial非依存でテストを動かすため分離)
├── config.py            設定(YAMLで上書き可能、既定値内蔵)
├── config/micromouse.yaml.example
├── requirements.txt
├── maze_files/          シミュレーション用の実大会迷路(Twilightから流用)
└── tests/               ユニットテスト+シミュレーションテスト(64件)
```

### 状態機械

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

遷移条件は曖昧さを避けるため明文化した(`micromouse/README.md` の表を参照)。
要点: ゴール判定は単一の `GoalRegion` 定義に対するセル座標の包含判定のみで行い、
探索終了時は必ずセル中心で停止する。最短経路は**未知壁=壁扱い**(安全側)で
計算し、探索時は**未知壁=通行可**(楽観)とする。

### 設計上の判断(Twilight からの変更点と理由)

1. **壁を3状態 enum(UNKNOWN/OPEN/WALL)に再設計**
   Twilight は known/observed の bool 2面持ち。等価だが指示書の要件に合わせ
   単一表現とし、隣接セル整合は `Maze.set_wall()` が常に両面更新して保証する。
2. **探索と自己位置管理の分離**
   Twilight の `decide_heading()` は壁更新+距離再計算+方位決定+位置更新が
   一体だった。Pose は状態機械が所有し、Explorer は純粋な意思決定のみ行う。
3. **スタートセルの壁も観測で決める**
   Twilight は「スタートセルの東壁あり」を決め打ちしていたが、実大会迷路には
   東が開いた面が存在し、シミュレーションで探索不能に陥るバグとして検出。
   スタート地点でもセンサ観測して迷路を更新する方式に変更した
   (「観測に基づいて迷路を更新し、その結果から行動を決める」という
   指示書の原則どおりの構造)。
4. **スレッド化せず同期ドライバ+中断コールバック**
   Twilight のスレッド版はボール検出との並行処理のため。マイクロマウスでは
   判断点間の処理が全て同期で足りるため、保守性を優先して簡素化した。
5. **シミュレータは壁の突き抜けを例外として検出**
   探索・走行アルゴリズムの整合性(「認識している迷路」と「実際の走行」の
   ずれ)をテストで機械的に検出できる。

### 安全動作

以下を検出したら走行より優先して **QSTP → MOT,0,0 → ERROR/EMERGENCY_STOP**:

- SEN 取得の連続失敗(リトライ含め5回)/ 走行コマンドの DONE タイムアウト
- バッテリー電圧低下(既定 6.5 V 未満、判断点ごとにチェック)
- センサ値異常(飽和・負値)
- L ボタン / Ctrl+C による中断
- 予期しない例外(捕捉して必ず停止してから再送出)

なお Daylight の mob には引数なし `STOP` コマンドが無い(`STOP,`のみ)ため、
無条件停止は `MOT,0,0` を使用している。

---

## 3. テスト結果

### 自動テスト(ハードウェア不要)

```
software/venv/bin/python3 -m unittest discover -s software/micromouse/tests -q
→ Ran 64 tests ... OK
```

- ユニットテスト: 壁の追加・隣接セル整合・迷路境界・ゴール判定・BFS/Flood Fill・
  最短経路・方向変換・180度旋回・行き止まり・SENパース・しきい値判定
- シミュレーションテスト(仮想迷路+仮想センサ+仮想走行):
  - 4×4迷路で全シーケンス完走、状態遷移順序の検証、走行ログ内容の検証
  - **全日本大会 1981年・1993年の16×16実迷路で完走**(最終位置がゴール中心で
    あること、学習した壁が真の迷路と一致することを検証)
  - 中断→EMERGENCY_STOP、低電圧→ERROR、ゴール封鎖迷路→ERROR(no path)
- 既存機能への影響なし: `default_app` 97件パス。
  ※ `software/ui/tests` に既存の失敗が1件あり(`test_sigterm_calls_cleanup` が
  `SystemExit` を捕捉していない)。今回の変更とは無関係(ui/ は未変更)。

### 実機確認(CM4上で実施、モータは回していない)

- mob は `/dev/ttyUSB0` で応答(`SEN` クエリに対し電圧 8.04 V、ジャイロ ≈ 0)。
- mob ファームウェア(現行 `mob.ino`)のコンパイルが通ることを確認
  (930 KB / 71%、arduino-cli)。

---

## 4. 残作業(ユーザー対応が必要)

1. **ESP32 ファームウェアの更新(必須)**
   実機に書き込まれているファームは旧版(SEN 7フィールド、FWD/STOP/TURN 非対応)。
   ```bash
   cd software/mob && make upload PORT=/dev/ttyUSB0
   ```
   書き込み後 `hw_test.py sen` で 11 フィールド応答を確認する。
2. **pyserial のインストール(必須)**
   venv への変更は控えたため未実施。
   ```bash
   software/venv/bin/pip install pyserial
   ```
3. **実機の段階的検証**(README の手順どおり)
   ```
   hw_test.py sen → walls(しきい値校正) → gcal → fwd → turn left/right/back → cycle
   ```
   壁しきい値(`wall_left/right/front_threshold`)は判断点(セル境界)に機体を
   置いた状態の値で `config/micromouse.yaml` に校正する。
4. **実迷路での走行**
   ```bash
   software/venv/bin/python3 software/micromouse/micromouse_app.py --port /dev/ttyUSB0
   ```
   R ボタンで開始、L ボタンで緊急停止。走行ログは `micromouse/logs/*.jsonl`。

## 5. 完了条件チェックリストとの対応

| 区分 | 項目 | 状態 |
|---|---|---|
| ソフトウェア | Maze/壁管理/隣接整合/位置/方向/ゴール判定/探索/最短経路/命令変換/状態機械 | ✅ 実装+テスト済み |
| ハードウェア | 壁センサ/エンコーダ取得 | ✅ SEN疎通確認済み(旧ファーム) |
| ハードウェア | モータ制御/直進/旋回/セル走行 | ⏳ ファーム更新後に `hw_test.py` で検証 |
| 実機 | 迷路走行/探索/ゴール/最短再走行/異常停止 | ⏳ シミュレーションでは全て確認済み。実迷路での検証待ち |

## 6. 高速化に向けた次のステップ(将来)

指示書の方針どおり初期実装は低速・確実性優先とした。基本動作の安定後:

- 探索速度・最短走行速度の引き上げ(`config` の速度パラメータのみで調整可能)
- 旋回を伴うセル通過のスラローム化(`cell_runner.py` に閉じて実装可能)
- 未知壁を考慮した探索打ち切り(全面探索せず最短候補経路のみ確定する戦略)
- 斜め走行・軌道最適化
