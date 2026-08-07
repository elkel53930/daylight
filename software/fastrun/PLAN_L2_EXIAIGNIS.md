# L2 高速走行の作り直し計画（ExiaIgnis 参考）

- 日付: 2026-08-07
- 進捗: Phase 2(kanayama 採用)・Phase 3(速度ランプ+統合チェーン)実機検証完了(2026-08-07)。

## 進捗ログ

### 2026-08-07 (Phase 3 統合チェーン: go_to → recenter_cell)
- `liner_move.py` に `go_to_and_recenter()` を追加: go_to で高速移動 → 停止 →
  到達後の物理向き(最終 heading)を SANG で再基準化 → `recenter_cell` でマス中心確定。
  到達後は odo_ang が走行中ドリフトするため、SANG で貼り直してから補正する設計。
- `verify_phase3.py`(検証スクリプト)を新規作成: (0,0) 北向きを壁上面補正で確定 →
  go_to(1,1) → recenter_cell。ゴールは (1,1)(X=W面・Y=S面、両軸に壁あり)。
- 実機結果: go_to は reached=True・max|hdg_err|=4.18°(発振なし)。到達時の位置ドリフトは
  Y約24mm・X約11mm 出ていたが、recenter_cell が両軸ともマス中心へ復帰:
  - recenter[y] face=S offset=-23.7mm → JOGFWD 23.7mm で修正 ✅
  - recenter[x] face=W offset=+11.2mm → JOGBACK 11.2mm で修正 ✅
- 所見: 高速移動→壁上面補正の統合チェーンが機能。go_to のドリフトが大きくても
  recenter_cell が5mm以内へ戻す、という狙い通りの動作。
- ⚠️ 留意: recenter 後は最後の X 軸面(W/E)を向いて終わるため、`go_to_and_recenter` の
  戻り pose.heading は recenter 後の実向きと一致しない。次移動前に SANG で貼り直す
  (Phase 5 ミッションループで扱う)。

### 2026-08-07 (ホイール直径修正後の再検証: kanayama w23)
- ホイール直径の重複定義が3箇所で食い違っていた(sensors.h=21.7mm / motion_controller.cpp・
  place_controller.cpp=23.4mm)のを、全て **23.0mm** に統一(ユーザー指示)。オドメトリ換算の
  ずれ(約8%)を解消。再ビルド・書き込み済み。
- kanayama(ky=0.004) を同一パターンで再走行 → 完走2.70s。旧直径と比較:

  | 指標 | kanayama(旧直径) | kanayama(w23) |
  |---|---|---|
  | 直進270mm max\|hdg_err\| | 1.03° | 0.40° |
  | スラローム max\|hdg_err\| | 2.69° | 2.46° |
  | 直進450mm max\|hdg_err\| | 4.35° | 4.53°(進入時のみ) |
  | 直進中 hdg_err 推移 | 4.35→即収束 | 4.53→即収束 |
  | 終点 ry(目標-540mm) | -515mm | -514mm |
  | 終点 dist | 25.9mm | 26.9mm |
  | 終点 hdg_err | -0.34° | -0.17° |

- 所見: 直進区間の heading 誤差が改善(1.03→0.40°、終点 -0.34→-0.17°)。オドメトリ換算の
  整合が取れた効果。スラローム出口の 4.5° スパイクは残るが即収束し発振なし。
  ⚠️ ホイール直径は3箇所重複定義(sensors.h / motion_controller.cpp / place_controller.cpp)。
  GEAR_RATIO 同様、変更時は必ず全箇所を合わせること。

### 2026-08-07 (Phase 2 実機検証: kanayama vs blend 比較)
- 検証スクリプト `verify_kanayama.py` を作成。WALL,0(壁追従無効)で Kanayama 効果のみ分離し、
  同じパターン((0,0)北向き → 直進270mm → 右90°スラローム → 直進450mm → (3,2))を
  `path_ky=0.0`(旧方式)と `0.004`(新方式)で各1回走行。走行前に壁上面補正(recenter_pose、
  recenter_cell 再利用)で (0,0) 中心・北向きを確定。
- 結果(#T ログ 54サンプル/走行、logs/verify_kanayama_*_20260807_*.csv):

  | 指標 | blend (ky=0) | kanayama (ky=0.004) |
  |---|---|---|
  | 完走 | ✅ 2.70s | ✅ 2.70s |
  | 直進270mm max\|hdg_err\| | 0.86° | 1.03° |
  | スラローム max\|hdg_err\| | 2.75° | 2.69° |
  | 直進450mm max\|hdg_err\| | 2.81° | 4.35°(進入時のみ) |
  | 直進450mm 中 hdg_err 推移 | 2.81→-1.20→+0.63→… 減衰振動 | 4.35→-0.46→-0.57→… 即収束 |
  | 終点 ry(目標-540mm) | -522(18mm手前) | -515(25mm手前) |
  | 終点 dist | 26.9mm | 25.9mm |

- 所見: 両方式とも発振なし・正常完走。Kanayama はスラローム出口直後に 4.35° のスパイクが
  1回あるが、以降 -0.3〜-0.6° で安定(旧方式は ±1.2° 程度の減衰振動を継続)。
  横ずれ(ry)は本コース(直進450mm+スラローム)では両方式で差なし。Kanayama の利点は
  長直進・外乱ありで出るため、本コースでは決め手に欠けた。
- ⚠️ 手順の注意(2026-08-07判明): 走行は連続実行しないこと。1走行後にロボットはゴール
  (3,2)・東向きで停止し、recenter_pose は「(0,0)・北向きに手で置いた」前提なので、
  次走行の前に必ず機体を (0,0) 中心・北向きへ手で置き直す(連続実行すると壁の無い方向を
  向いて recenter が遠距離JOGFWDを繰り返し、recenter 失敗で中止)。

### 2026-08-07 (Phase 2 実装・書き込み)
- `path_controller.cpp` に Kanayama 式横位置復元力を実装(`path_ky>0` で有効)。
  機体フレームでのターゲット横ずれ e_y に比例する heading バイアスを毎tick常時重畳。
  `path_ky=0` なら従来のベアリングブレンドへフォールバック(後方互換)。
- `params.h/cpp` に `path_ky`(既定 0.004 rad/mm)・`path_ky_max`(0.3 rad)追加。
- リモートビルド成功・`make upload` で書き込み完了。PGET で `path_ky=0.004`/`path_ky_max=0.3`
  を確認(NVS未保存のためビルド既定が有効)。
- ⏳ 次: Phase 3 へ(速度ランプの実機詰め、`recenter_cell` との統合チェーン)。
  Kanayama の効果を見るためには長直進または外乱(壁追従ON等)を含むパターンで再比較するか、
  決定的な差異が出る場面を選定してから進める。
- 対象: Liner の既知迷路・点対点高速移動（`software/fastrun/` の L2 + `software/mob/` の ESP32 制御）
- ゴール: **入力（迷路 + 現在座標 + 向き NSWE + ゴール座標）→ 最短時間で到達する走行**。
  経路生成は RPi/Python、リアルタイム制御は ESP32（現構成のまま）。
  制御設計の思想は ExiaIgnis（https://github.com/Naophis/ExiaIgnis）を参考にする。

## 1. 現状と課題

### 現在できること（L2、2026-08-04 実機検証済み）
- `planner.py`: (セル, 向き) 状態のダイクストラ → ターン時間ペナルティ込みの最短時間経路。
  `plan()` が pattern.py の Straight / Slalom 区間列を返す。
- `liner_move.go_to()`: 初手方向転換は超信地旋回 → `send_pattern`（PCLEAR/PADD/PRUN）→
  PATTERN 走行。到達は `#T` テレメトリ（seg_index 完了）で検出。実機 (0,3,W)→(3,3)
  @220mm/s 成功、max|hdg_err|=1.9°。
- mob 側 `path_controller.cpp`: 仮想ターゲット追従（pure pursuit 風）。
  - 角度誤差: `target_heading - robot_theta` を基準に、dist が開くとベアリング角へブレンド
    （`path_blend_mm=40`）。`path_gate_mm=200` で離れすぎたらターゲット停止（復帰待ち）。
  - 前進: FF（`kf_duty_per_mps * v`）+ 距離誤差 P（`path_kp_fwd`）。
  - 旋回: 幾何 FF（`path_kf_ang * omega_ff`）+ 方位誤差 P（`path_kp_ang`）+ 角速度誤差 D
    （`path_kd_ang * rate_error`）。
  - 直進中、両側壁ありなら `path_wall_kp * (rs-ls)` を heading バイアスとして重畳（壁追従、
    既定 0.05 で有効）。

### 課題（ExiaIgnis と比較して）
1. **直進の真っ直ぐさ/スラロームの追従精度がゲイン依存**で、速度を上げると横ずれが増える。
   位置復元力（ベアリングブレンド）は「dist が開いてから」働く遅めの機構。
2. **位置・向きの推定がオドメトリ（エンコーダ + ジャイロ積分）のみ**。ExiaIgnis の Kalman 推定
   （IMU + エンコーダ + 壁センサ融合）がない。ドリフトは走行ごとにカメラ（壁上面補正 L1）で
   リセットする前提だが、走行中はフィードバックが弱い。
3. **経路探索はターン時間ペナルティのみ**で、速度プロファイル・区間ごとの時間見積もりが
   粗い（巡航固定）。斜め走行（45°）未対応（planner は拡張可能な設計）。
4. **外乱・加減速の扱い**: 台形プロファイルはあるが、MPC 的な「残距離ベースの最適速度」や
   外乱オブザーバーはない。

## 2. ExiaIgnis の制御設計（参考にする点）

`src/planning/` の 3 層を調査済み。Liner に移植する価値が高いのは以下。

### (a) EgoEstimator（状態推定）
- エンコーダ速度・ジャイロ角速度・（あるなら）壁センサ距離を Kalman で融合。
- 車体速度 v、ヨー角速度 ω を推定し、そこから x/y/θ を積分。
- → Liner では「ジャイロ角速度 + エンコーダ（左右輪 → v）」を融合する軽量推定器を
  ESP32 に追加するのが本筋。**ただし 1kHz で組むと実装・チューニングコストが高い**ため、
  第一段階は現行のオドメトリ（`sensors_.get_distance()/get_angle()`）のままとし、
  Kalman 化は Phase 後半の任意タスクとする。

### (b) TrajectoryGenerator（目標軌道生成）
- 直進: 残距離ベースの台形速度プロファイル（Liner の `advance_straight` と同思想、既存）。
- スラローム/曲線: **Kanayama 制御**（位置誤差（e_x, e_y, e_θ）から
  v = v_ref·cos(e_θ) + k_x·e_x、ω = ω_ref + v_ref·(k_y·e_y + k_θ·e_θ) で復元）を
  目標速度・目標角速度の発生に使う。
  - → Liner の「仮想ターゲット追従」は Kanayama の e_x/e_y を「dist_to_target − path_follow_mm」
    とベアリング角で代替したもの。**Kanayama を正しく入れる = スラローム中の横ずれ復元が
    幾何学的に正しくなり、高速化しても発散しにくい**。これが最大の改善点。

### (c) ControlLaw（追従）
- カスケード PID + MPC（計算トルク）+ 外乱オブザーバー。
- → Liner は 1kHz・ESP32 で現行 PID が既に実装済み。**外乱オブザーバー（1次遅れの
  逆動力学フィードバック）を旋回系に追加**すると、バッテリ電圧変動・路面差による
  外乱を打ち消せて直進の真っ直ぐさが上がる。ただしまずは現行ゲインのままで速度を
  上げ、限界を見極めてから。

## 3. 方針（スコープ判断）

- **経路生成 = RPi（Python）**: 現行 `planner.py` を拡張。斜め走行・速度プロファイルも
  PC 側で決めて区間列にする（ESP32 は受け取った区間を追従するだけ = 現構成のまま）。
- **リアルタイム制御 = ESP32（C++）**: `path_controller.cpp` を強化。Kanayama 相当の
  位置復元力を導入し、追従ゲート・ブレンドは維持 or 置換。
- **やらないこと（このフェーズ）**: Kalman 化・MPC・外乱オブザーバー（Phase 6 で任意）。
  探索（Eiffel が供給）・斜め走行（Phase 5）。壁上面補正（L1）は完成済みなので再利用。

## 4. 実装フェーズ

### Phase 1: 到達検知・テレメトリの安定化（現状確認）
- `go_to()` の `#T` 監視を実機ログで検証。seg_index 完了 + heading_error 発振監視が
  正しく動くか、低速（220mm/s）で再確認。
- 到達時のドリフト量を実測（オーバーシュートの抑制要否の判断材料）。
- 成果物: 実測ログ + 現状課題の確定。

### Phase 2: path_controller への位置復元力（Kanayama 的 e_x/e_y）導入 ✅(2026-08-07 実機検証完了)
`software/mob/path_controller.cpp` を改修:
- 現在の「ベアリングブレンド」を、**機体を原点・向き X+ にとったフレームの
  位置誤差（e_x, e_y）**ベースの復元力へ置き換える:
  - e_x = dist_to_target − path_follow_mm（前後誤差、既存の dist_error）
  - e_y = 横誤差（ターゲット方向と機体向きのずれから算出）
  - ω 指令 += k_ey · e_y（横ずれを向き制御で戻す）+ k_eθ · (target_heading − theta)
  - v 指令 = v_ff + k_ex · e_x（既存の前進 P を流用）
- これでスラローム中も「曲線の内側/外側へずれたら軌道へ戻る」力が働く。
- パラメータ追加: `path_kanayama_ky`, `path_kanayama_ktheta`（`params.cpp` に追加、
  PSET/PSAVE でライブ調整可）。
- **ベアリングブレンド + ゲートは後方互換のためパラメータで切替可能に**（既定は新方式）。
- 検証: 既存 `DEFAULT_TEST_PATTERN`（180°Uターン込み）を実機で走り、追従誤差 dist の
  最大値・復帰が旧方式より良化していることを `#T` で確認。

### Phase 3: 速度ランプ（moderate → 高速） ✅(2026-08-07 実機検証完了)
- `planner.py` の `PlannerConfig` を実機結果で詰める: 直進 400mm/s 目標、スラローム 360mm/s。
- 各 Phase で必ず `#T` の heading_error・dist を確認し発振を見逃さない
  （CLAUDE.md 開発ルール）。
- `go_to` 到達後に `recenter_cell`（L1 壁上面補正）を呼ぶ統合チェーンを確立。

### Phase 4: 経路探索の時間見積もり改善 + 斜め走行
- `planner.py`: ダイクストラのエッジコストを「直進時間（巡航速度から加速分を考慮）+
  ターン時間」へ精緻化（Phase 4a、完了）。
- **Phase 4b(2026-08-07、実装完了・実機検証完了 ✅): 8方向拡張(本格的)**
  - `geometry.Direction` を8値化(N/NE/E/SE/S/SW/W/NW、45°刻み・時計回り)。
    `turned`/`turn_between` のステップは 45° 単位へ(90°=±2、180°=+4)。
  - 斜め移動は「両隣接セルへの直交移動が共に可能」を条件に許可(`maze.can_move`、
    角を切る通行。外周へは不可)。斜め1セル=√2×180mm(`DIAG_CELL_MM`)。
  - 区間生成: 45/90/135°を単一スラローム(接線長 R·tan(θ/2)で前後直進を短縮)、
    180°は従来どおり90°×2。斜め直進 run の距離は √2 倍。
  - 探索コストのターンペナルティは角度比例(45°=半分、135°=1.5倍、180°=uturn)。
  - `explorer`/`floodfill` は探索走行のため従来どおり直交4方向で動作(斜め除外、
    `mapping` の左右は ±2 へ)。`liner_pose.direction_to_gyro_deg` に斜め方位追加。
  - 注意: 実機は mob の SLALOM が任意角度対応済みなので 45/135° も送れる。
    go_to の初手超信地旋回・SANG も斜め方位対応。
  - テスト 95件パス(斜めカットの壁判定・45°/135°/180°区間・時間見積りを追加)。
  - **実機検証(2026-08-07)**: 検証パターン「(0,0)中心・北向き → 直進232.8mm →
    45°右スラローム(R=90, 220mm/s) → 斜め直進344.5mm(NE)」を実行
    (`verify_diag45.py`、直進250mm/s)。**完了 reached=True、発振なし
    (hdg_err 符号反転0回)**。区間別 max|hdg_err|: seg0(直進)=0.46°、
    seg1(スラローム)=1.60°、seg2(斜め直進)=2.81°。最終向き -43.7°(目標 -45°、
    誤差1.3°)。終端世界 (346,638)mm、期待 (360,630)mm から約16mm(ドリフト+
    終端検出タイミング)。壁クリアランスはシミュレーション通り最小63.6mmを確保。
    事前に `diag_sim.py`(新規)で 45°スラローム後の最小クリアランスを検証済み。

### Phase 4c(2026-08-07〜): 走行精度検証の周回ループパターン(新規)
- 目的: スラローム・斜め走行・長直進の追従精度と**閉ループ誤差**(発着一致)を
  一度に測る。`verify_loop.py`(新規)。
- 経路(diag_sim.py で全壁クリアランス最小63.6mm・終点(90,90)を検証済み):
  (0,0)中心・北 → 直進232.8 → 45°右スラローム → 斜め直進344.5(NE) → (360,630)
  → 45°右スラローム → 直進116.4(東) → 90°右スラローム → 直進46.4(南) →
  90°右スラローム → 直進360(西) → 90°左スラローム → 直進250(南) → (0,0)中心・南。
  全旋回角の和=360°(=弧長565.5mm)+直進1350.1mmの閉ループ。
- 指標: 最終 #T の rx/ry(理想0,0)、rtheta(理想180°)、セグメント別 max|hdg_err|、
  hdg_err 符号反転(発振)。完走後は俯瞰投稿で発着位置を目視確認。
- 走行前は (0,0) 中心・北向きに手で置く。完走後は (0,0) 中心・南向きで停止。
  実機検証は未実施(低速250/220から開始予定)。

### Phase 5: L3 ミッションループ統合（待機→go_to→recenter_cell→待機）
- `liner_mission.py`（仮）: ゴール列を連続実行。各ゴール到達 ≤5mm を俯瞰で確認。

### Phase 6（任意）: 外乱オブザーバー / Kalman 推定
- 旋回系に 1次遅れ外乱オブザーバーを追加して直進・スラロームの外乱耐性を上げる。
- Kalman 化は 1kHz 実装コストが高いため、速度・角速度推定のみ限定導入の選択肢。

## 5. 変更ファイル一覧

| ファイル | 変更内容 |
|---|---|
| `software/mob/path_controller.cpp/.h` | Kanayama 的位置復元力の導入（Phase 2） |
| `software/mob/params.cpp/.h` | `path_kanayama_ky`/`path_kanayama_ktheta` 等を追加 |
| `software/fastrun/planner.py` | 時間見積もり精緻化・斜め対応（Phase 4） |
| `software/fastrun/liner_move.py` | 速度パラメータ・到達後 recenter_cell 連携（Phase 3） |
| `software/mob/pattern.py` | 斜め区間（45° スラローム）追加（Phase 4） |
| `software/fastrun/liner_mission.py`（新規） | ミッションループ統合（Phase 5） |

## 6. 検証手順

1. ユニットテスト: `software/venv/bin/python3 -m unittest discover -s software/fastrun/tests -q`
   （`path_controller` の変更は ESP32 側なので Python テスト対象外、実機確認が必要）。
2. 実機走行は**動かす1秒前にブザー**（`notify.warn_before_move`）→ 走行 → `#T` ログ確認
   （発振なし）→ 俯瞰カメラで真値確認（`overhead.capture_and_post`）。
3. 各フェーズを低速から順に上げ、必ず前段の実機検証を通過してから進む。

## 7. 注意（CLAUDE.md / HANDOFF.md から抜粋）
- `params.cpp` を書き換えても **NVS 保存値が優先**される。PSET で RAM 変更 → PSAVE。
- `make` は cwd が移動するので、スクリプトは絶対パスで叩く。
- RANG/SANG/GCAL は TURN 角度保持中に出さない（暴走）。必ず MOT,0,0 で抜けてから。
- `go_to` で RANG/RDST は呼ばない（path_controller が自己参照、odo_ang は絶対のまま）。
- 横壁センサ利用は壁に近づきすぎない。近接時は WALL,0。
