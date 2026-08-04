# fastrun 引き継ぎ(2026-08-05)

次のAIエージェント/開発者向けの現状まとめ。前提知識(ハード特性・用語・安全
ルール)はリポジトリ直下 `CLAUDE.md` と memory を必ず先に読むこと。ここは
**いま何ができていて、次に何をすべきか**と**ロボットの動かし方**に絞る。

- 作業ブランチ: `mob-place-hold`(master には触れない)
- 直近コミット: `3a404e1`(JOG高速化)/ `0016374`(壁上面補正に距離認識モデル統合)
- 作業ツリーの未コミット: `software/camera/camera_test.py`(無関係な差分、**コミット
  しない**)、`.claude/`(未追跡、コミットしない)。それ以外は全てコミット済み。

---

## 1. Liner のミッション(再掲)

待機 → ゴール座標受領 → 既知迷路で高速移動(L2) → 到達 → **カメラで壁上面補正
してマス中心へ戻す(L1)** → 待機 → 繰り返し。迷路情報・ボール座標は視覚エージェント
Eiffel から既知で与えられる前提。探索走行は不要。開発計画は memory `liner-dev-plan`
(L0足場→L1マス中心化→L2点対点高速移動→L3統合→L4高速化→L5 Eiffel+ボール)。

---

## 2. 壁上面補正(L1)の現在の設計 ★今回の主対象

カメラで前方の**赤壁の下端エッジ**(赤壁と白床/内側の境界)を検出し、機体を
マス中心(壁まで90mm)へ戻す。ヨー(向き)成分と前後位置成分がある。

### 使用モジュール
- `wall_bottom.py` … `detect_red_band_bottom_edge(bgr)` → `RedBandEdge(slope,
  intercept, inlier_count, residual_std)`。`software/vision/wall.py`(HSV+RANSAC)を
  ラップ。**下端エッジ**は背景・照明変化に強い(上端は暗所の赤っぽい光で誤検出した)。
- `camera_model.py` ★新規 … **距離認識モデル**。治具較正(75〜115mmを5mm刻み、
  ±18°)で、row↔距離・yaw gain↔距離・straight↔距離 が**いずれも非線形**(近いほど
  高感度)と判明したため、区分線形補間で距離依存を反映する。主関数:
  - `distance_from_row(row_calib) -> 距離mm`(範囲外はクランプ)
  - `estimate(row_calib, slope_deg) -> (距離mm, ヨーdeg)`(ヨー正=左/CCW)
  - `forward_offset_mm(row_calib) -> 90mmからの前後ずれ`(正=中心より前=壁に近い)
  - `is_row_in_range(row_calib) -> bool`(較正範囲 row 607〜1141 = 75〜115mm)
  - 較正点は `_DIST_ROW`(9点)・`_DIST_GAIN`(75/90/115mm)。**再較正時はここを更新**。
- `recenter.measure_wall(cam, n=6) -> WallMeasure` … n枚撮影し下端エッジを検出、
  `camera_model.estimate` で (距離, ヨー) を同時推定、清浄フレーム(res<閾)の中央値で
  頑健化。`WallMeasure(dist_mm, yaw_deg, offset_mm, res_px, n_clean, ok)`。`ok` は
  「較正範囲内 かつ 過半数フレーム清浄」= 一発補正に使ってよいか。
- `liner_center.center_axis(link, cam, face)` … **オープンループ**(反復FBなし、
  2026-08-05ユーザー指示)。
  1. `turn_to` で face をおおまかに向く(ジャイロ)
  2. 【測定1】ヨーを測り `JOGTURN` で一発旋回 → `MOT,0,0` 停止 → `SANG` で絶対方位確定
  3. 【測定2】正対後に前後距離を測り `JOGFWD`/`JOGBACK` で一発移動して90mmへ
- `liner_center.recenter_cell(link, cam, maze, pose)` … 現在セルの壁がある軸を
  center_axis。壁が無い軸は近傍マスへ移って補正(フォールバックは段階実装中)。

### カメラ撮影の要点(`camera_align.OnboardCamera`)
- **VGA(768×432)で撮影**するが、`raw={"size":(2304,1296)}` でセンサモードを較正時と
  同一FOVに固定(これを外すと別モードが選ばれ row が1.36倍ずれる)。検出後 row は
  `×(CALIB_HEIGHT/h)=×3` で較正基準(1296)へ換算。
- **Futabaサーボは必ず0度**で撮影。再現性のため一度20度へ振ってから0度へ戻す
  (`OnboardCamera` が実施)。**0度未満は厳禁(ハード故障)**。
- **静止時のみ撮影**(走行中はブレて精度が出ない)。露出安定を待つ。

### 検証状況(重要)
- ✅ ユニットテスト全82件パス(`camera_model` 12件含む)。
- ✅ `camera_model` は治具較正で妥当性確認(ヨー RMS 0.58°/±18°、小角度ほぼ完全)。
- ⚠️ **統合後の end-to-end 実機センタリング(measure_wall→center_axis の一連)は
  未検証**。個々の部品(距離認識モデル、JOG、下端エッジ検出)は検証済みだが、
  通しの実走はまだ。**次の実機ステップの最優先はこれ**(下記コマンド参照)。

---

## 3. JOG高速化(今回実施・検証済み)

`JOGFWD`/`JOGBACK` が本体≈25mm/sと遅く低速域で制御性が悪かったため、位置ホールド
外側ループのゲインを引き上げた(`mob/params.cpp`):
- `place_pos_kp` 2.0→**6.0**、`place_pos_max_mps` 0.05→**0.14**(和=0.14 → 本体巡航≈70mm/s)
- 60mm往復が総2.3s→**1.6s**、巡航≈75mm/s(旧の約3倍)、オーバーシュート無し・逆行0。
- 共用先の `HOLD` 静止保持(p2p 0.11mm)・旋回後保持(並進 p2p 0.26mm)も発振なし。

### ⚠️ params の永続化に関する重大な注意(NVS優先)
mob は起動時 `params_load()` が **NVS(PSAVE保存値)をコンパイル既定より優先**して
読む。よって **`params.cpp` を書き換えて `make upload` しただけでは、既に NVS に
その項目が保存されている機体には反映されない**(今回 `place_pos_kp` が NVS の 2.0 に
上書きされていて気づいた)。反映手順:
```
PSET,<name>,<value>   # RAMを即変更
PSAVE                 # 全paramを現在RAM値でNVSへ保存(応答は DONE)
```
今回の値は実機NVSへ PSAVE 済み(リセット後も 6.0/0.14 を確認)。paramsの確認は
`PGET,<name>`(応答 `PVAL,<name>,<value>`)または `PGET`(全列挙)。

---

## 4. ロボットの動かし方(実務)

### シリアル接続
- `/dev/ttyUSB0`、**3,000,000 bps**。`software/fastrun/mob_link.py` の `MobLink` を使う
  (context manager、`.send(str)`/`.wait_for(prefix,timeout)`/`.read_sen()`/
  `.gyro_calibrate()`/`.stop()`)。接続時にESP32が自動リセットされる。
- `make upload` 直後の初回接続は SEN を取りこぼしやすい → 落ちたら再実行。

### 移動系コマンド(現行、CLAUDE.md も参照)
- `MOT,<r>,<l>` 速度[mm/s]即時指令(完了通知なし)。停止は `MOT,0,0`。
- `DUTY,<r>,<l>` 生duty(−1023〜1023)。
- `HOLD` その場静止制御開始。
- `TURN,<rad>`(正=左/CCW)超信地旋回(角度保持継続、DONE無し)。
- `JOGFWD,<mm>`/`JOGBACK,<mm>` ヨー保持で前/後へ低速並進、**到達でDONE**。
- `JOGTURN,<rad>` 台形その場旋回、**到達でDONE**。
- ⚠️ `RANG`/`SANG`/`GCAL` は **TURN角度保持中に出さない**。必ず先に `MOT,0,0` で
  保持を抜けてから(保持中に出すと機体が暴走した実機事象あり)。

### 安全ルール(必須、CLAUDE.md「開発の進め方」)
- **動かす1秒前にブザー**を鳴らす: `from notify import warn_before_move; warn_before_move(1.0)`。
- **毎走行、位置・速度・角度・角速度のログを確認し発振がないか点検**(振動継続は
  ハード故障・精度低下の原因)。
- 横壁センサは壁に近づきすぎない(近接時は `WALL,0`)。カメラが唯一の絶対基準。
- 位置ずれ5mm程度は許容、5mm超は避ける。
- 超信地旋回は**左右バランス**(電源ケーブルのよじれ対策)。`recenter.net_rotation_deg()`
  で正味回転を追跡、偏ったら `recenter.unwind_cable()`。

### 壁上面補正を実機で実行(次の最優先ステップ)
```bash
# 機体を既知セル・既知向きに置き、目の前(face方向)に壁がある状態で:
software/venv/bin/python3 software/fastrun/recenter_cli.py \
    --port /dev/ttyUSB0 --cell 0,0 --dir N --maze dev
```
`recenter_cli.py` は `recenter_cell` を呼ぶ。**実行前に俯瞰カメラ(x13u C270)で
真値を確認**し、補正前後の位置を VGA(640×480)画像で Discord に投稿すること
(memory `overhead-camera-c270`、`robot-dev-rules`)。まずは `center_axis` 単体を
1面で試し、measure_wall の推定(距離・ヨー・ok)が妥当か、一発補正で残差が5mm以内に
入るかを確認するのがよい。ズレが大きい/ok=False が多い場合は `camera_model.py` の
較正点(`_DIST_ROW`/`_DIST_GAIN`)を治具で再取得する。

### テスト
```bash
software/venv/bin/python3 -m unittest discover -s software/fastrun/tests -q
```
(既知の無関係な失敗: `software/ui/tests` の test_sigterm_calls_cleanup。fastrunは全82件パス)

### ファーム(mob)ビルド/書き込み
`make build`/`make upload` は x13u へオフロード(memory `mob-remote-build`)。書き込みは
許可済み(memory `user-permits-mob-upload`)。**paramsは上記NVS注意に従い PSET+PSAVE**。

---

## 5. 次にやること(優先順)
1. **壁上面補正の end-to-end 実機検証**(measure_wall→center_axis の通し)。俯瞰で真値
   確認しながら1面ずつ。残差5mm以内を目標。ずれたら camera_model 較正点を再取得。
2. `recenter_cell` の近傍マス・フォールバック経路の実装/検証(壁が片軸しか無いセル)。
3. L2(点対点高速移動、`fastrun` の path/planner)との統合(memory `fastrun-project`)。

## 6. 便利スクリプト(scratchpad、参考)
セッションのscratchpadに JOG検証スクリプトあり(jog_speed_test.py / jog_profile.py /
jog_sweep.py / hold_turn_check.py)。掃引はPSETでライブ、リフラッシュ不要。手法は
本ドキュメント3章の通り。
