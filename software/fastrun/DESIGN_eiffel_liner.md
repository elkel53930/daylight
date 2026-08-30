# Eiffel → Liner 座標移動連携 設計(2026-08-30)

視覚エージェント **Eiffel** が把握した迷路情報をもとに、**Liner** を指定座標まで
移動させるための設計。memory `liner-dev-plan` の **L3(ミッションループ統合)** に相当。
前提知識(ハード特性・用語・安全ルール)は `CLAUDE.md`・`HANDOFF.md`・memory を先に読むこと。

---

## 1. Eiffel データ契約(実機 `two-unit-network` ブランチで確定)

Eiffel の各号機が Flask HTTP サーバ(ポート 5000)を立てる。**1号機(role=primary,
172.20.10.4)が2号機(role=secondary, 172.20.10.5)を zeroconf で発見し、結果を集約**する。
Liner は **1号機の `GET /status.json` の `combined` フィールドだけ**を読む。

```
GET http://172.20.10.4:5000/status.json  (role=primary)
{
  ready, last_update, last_error, role,          # role: standalone/primary/secondary
  maze_cols, maze_rows, threshold, cells, balls, # ← 1号機ローカルのみ。Linerは使わない
  combined: {                                    # ★Liner が使うのはこれ(2台マージ済み)
    maze_cols: 16, maze_rows: 8,
    cells: [ {col, row,
              walls:       {N,E,S,W}(bool),
              walls_valid: {N,E,S,W}(bool)}, … ],  # 128セル、グローバル座標
    balls: [ {col, row, yellow_frac}, … ],
    peer_last_update                             # 2号機の最終更新時刻。None=2号機未接続
  }
}
```
未準備時: `{ready:false, last_error, role}`。

### 座標系(重要: Liner と完全一致 → 変換不要)
- 原点 (0,0) = **南西隅**(カメラ直下=スタート隅)。**col = 東方向 = Liner の cx**、
  **row = 北方向 = Liner の cy**。セル (col,row) = Liner の LinerPose(cx,cy)。
- 方位 N/E/S/W も Liner の `geometry.Direction` と同名・同義。

### 品質と堅牢性に直結する事実(Eiffel ソースで確認)
1. **`combined` は常にグローバル全域(16×8=128セル)を返す**。どちらの号機からも
   **未受信のマスは `walls=全false / walls_valid=全false`(未検出プレースホルダ)** で埋まる。
2. → **2号機オフライン時は東半分(col 8..15)が「壁なし・valid全false」に化ける**。
   値だけ見ると「開放」に見えるが実際は**未知**。`walls_valid` を無視すると壁へ突っ込む。
3. `peer_last_update = None` は **2号機未接続**。値があれば東半分の鮮度判定に使える。
4. 両号機が揃うと**内壁の invalid=0 / walls=True&valid=False=0** の非常にクリーンなマップ
   (無効辺は外周4隅のみ。WallMap が外周を常に壁扱いするので無害)。
5. `balls` はフレームごとに個数が揺れる(検出ノイズ)。ボール利用(後回し)は時間フィルタ前提。

---

## 2. 採用方針(ユーザー決定 2026-08-30)

- **迷路情報の一次ソースは Eiffel**。毎ミッション `combined` から `WallMap` を構築する。
- **迷路は 16×8**(cols=16, rows=8)。`combined.maze_cols/rows` を正とする。
- **Liner のスタートセル・向きは迷路ごとに変わる → YAML で設定**(§4)。
- **`walls_valid=True` の辺だけ信頼**。invalid 辺は「未知」。走行前に、計画経路が通る範囲の
  全辺が valid になるまでポーリングし、揃わなければ理由付きで FAIL(`peer_last_update=None`
  は即エラー材料)。

---

## 3. モジュール構成

```
eiffel_client.py   ★新規  Eiffel /status.json 取得 → combined → WallMap 変換(stdlib urllib のみ)
drive_runner.py    ★新規  verify_cells.run_once の走行・監視を関数化(挙動不変で切り出し)
mission.py         ★新規  状態機械: SYNC_MAP→PLAN→DRIVE→RECENTER→IDLE
mission.yaml       ★新規  設定(Eiffel接続/スタート姿勢/走行速度/衝突しきい値)

既存流用:
  maze.WallMap / geometry.Direction / liner_pose.LinerPose
  planner.plan(wm,start,start_dir,goal,cfg) / find_path
  liner_center.recenter_cell(壁上面補正 L1)
  notify.warn_before_move / recenter.turn_to,net_rotation_deg,unwind_cable
```

### 3.1 eiffel_client.py(本コミットで実装)
- `fetch_status(host,port,timeout_s) -> dict` : HTTP GET。接続不可/timeout/非200/JSON不正は
  `EiffelUnavailable`。
- `snapshot_from_status(status, valid_policy) -> EiffelSnapshot` : `combined`(無ければ
  standalone のトップレベル)を取り出し `WallMap` を構築。
- `fetch_snapshot(...) -> EiffelSnapshot` : 上記2つの合成。`ready:false` は `EiffelUnavailable`。
- `EiffelSnapshot(wm, cols, rows, balls, last_update, peer_last_update, role,
  invalid_inner_edges, undetected_cells, raw)` と `region_valid(cells)` / `all_valid()`。
- **WallMap 構築ポリシー**(`valid_policy`):
  - `"strict"`: `walls[d] and walls_valid[d]` の辺だけ壁を立てる(未知は開放のまま)。
  - `"conservative"`(既定): 上記に加え、**内辺で walls_valid[d]=False の辺は壁を立てて封鎖**
    (未知マスへ経路が伸びない=安全側)。ゲートを通さず走っても壁へ突っ込まない保険。
  - 外周辺の invalid は無視(WallMap が外周を常に壁扱い)。

### 3.2 drive_runner.py(次コミット)
`verify_cells.run_once` の GCAL→`WALL,0`→スタート recenter→`send_pattern`→`#T`/`#COLLIDE`
監視→完了/衝突判定→CSVログ を `drive_segments(link, segs) -> DriveResult{reached,
collision, log_path}` に括り出す。verify_cells もこれを呼ぶ形にリファクタ(挙動不変)。

### 3.3 mission.py(次々コミット)
```
IDLE ─[ゴール受領]→ SYNC_MAP → PLAN → DRIVE → RECENTER → IDLE
                       │(unavail)  │(到達不能) │(#COLLIDE)
                       └ retry     └ FAIL     └ RECOVER → PLAN(再計画)
```
- SYNC_MAP: `fetch_snapshot`。not ready/stale はバックオフ再試行。経路範囲が valid 揃うまで待つ。
- PLAN: `plan(wm, pose.cell, pose.heading, goal)`。`find_path` 空 → 到達不能 FAIL。
- DRIVE: 安全手順(ブザー1秒前・毎走行ログ点検・`WALL,0`)→ `drive_segments`。
- RECENTER: `recenter_cell` でヨー+X+Y をマス中心へ(壁が無い軸は近傍マス補正)。pose 確定。
- RECOVER: 静的迷路での衝突=自己位置ドリフト。停止→俯瞰(開発時)/搭載カメラで再ローカライズ
  →`SANG` 貼り直し→PLAN 再計画。連続失敗で中止。
- pose 初期値は YAML の start(§4)。以後は RECENTER 後の確定 pose を保持。

---

## 4. mission.yaml(スタート姿勢は迷路依存)

```yaml
eiffel:
  host: 172.20.10.4        # 1号機(集約先)
  port: 5000
  timeout_s: 2.0

maze:
  cols: 16                 # combined.maze_cols を正とし、これは期待値検証用
  rows: 8

liner:
  start:
    cell: [0, 7]           # 迷路ごとに変更(自陣スタート)。現在=北西隅(0,7)南向き
    heading: S             # N / E / S / W

map_sync:
  valid_policy: conservative  # strict / conservative
  poll_interval_s: 0.3
  poll_timeout_s: 5.0

drive:                     # verify_cells 実測の確定値
  straight_mmps: 700
  slalom_mmps: 550
  collide_dist_mm: 150     # firmware(PSET)と必ず揃える(CLAUDE.md参照)
  collide_ang_rad: 0.7
```

---

## 5. エッジケース

| 事象 | 対応 |
|---|---|
| 接続不可 / timeout / `ready:false` | `EiffelUnavailable`。mission はバックオフ再試行、上限で FAIL |
| `peer_last_update=None`(2号機未接続) | 東半分が未検出。東を通る経路なら FAIL(理由明示) |
| 経路範囲に invalid 辺が残る | poll_timeout まで待ち、揃わねば FAIL |
| `combined.maze_cols/rows` が YAML 期待値と不一致 | 警告し combined 側を採用 |
| 経路なし(`find_path` 空) | 到達不能 FAIL |
| `#COLLIDE` / `#T` 逸脱 | 停止→RECOVER→再計画。連続失敗で中止 |
| role=secondary/standalone に接続 | secondary はエラー、standalone はトップレベルを全域として採用 |

---

## 6. 段階的実装・検証計画

1. **`eiffel_client.py` + 単体テスト**(本コミット。ハード不要)。実 `/status.json` を
   `tests/data/eiffel_status_primary.json` に保存済み。2号機オフライン(未検出東半分)は
   テスト内で合成して検証。実機 Eiffel への `fetch_snapshot` 疎通も1回確認。
2. **`drive_runner.py` 切り出し**(verify_cells リファクタ、テスト挙動不変)。
3. **`mission.py --dry-run`**(fetch→plan→区間表示のみ、走らない)。俯瞰/`preview.jpg` と照合。
4. **実機 end-to-end(開発4×4)**: 既知配置で start→goal 走破→recenter→俯瞰VGAをDiscord投稿。
   ブザー1秒前・毎走行ログ点検厳守。1マス直進→L字→対角の順。
5. 異常系(接続断・stale・到達不能・衝突)を誘発し RECOVER/FAIL を確認。
