"""eiffel_client.py — Eiffel(視覚エージェント)から迷路情報を取得し WallMap へ変換(2026-08-30)。

`two-unit-network` ブランチの Eiffel 1号機(role=primary, 既定 172.20.10.4:5000)が公開する
`GET /status.json` の `combined` フィールド(2台マージ済み・グローバル座標 16×8)を読む。
標準ライブラリ(urllib)のみ。新規依存を追加しない。設計は `DESIGN_eiffel_liner.md`。

契約(2026-08-30 実機 two-unit-network で確認):
    {ready, last_update, last_error, role, maze_cols, maze_rows, threshold,
     cells, balls,                       # ← 1号機ローカルのみ。Liner は使わない
     combined: {maze_cols, maze_rows,
                cells: [{col, row, walls:{N,E,S,W}(bool),
                         walls_valid:{N,E,S,W}(bool)}],   # グローバル座標
                balls: [{col, row, yellow_frac}],
                peer_last_update}}       # 2号機の最終更新時刻。None=2号機未接続

座標系は Liner と一致(原点=南西、col=cx=東、row=cy=北)。変換不要。

未検出マスは walls=全 false / walls_valid=全 false で埋められる(2号機オフライン時は
東半分がこうなる)。値だけ見ると「開放」に見えるが実際は「未知」。よって WallMap には
walls_valid=True の辺だけを立て、invalid は「未知」として扱う(valid_policy 参照)。
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence, Tuple

from geometry import Direction
from maze import WallMap

DEFAULT_HOST = "172.20.10.4"
DEFAULT_PORT = 5000
DEFAULT_TIMEOUT_S = 2.0

# combined.cells / walls の向きキー。Direction と同名(N/E/S/W)。
_DIR_KEYS = ("N", "E", "S", "W")


class EiffelError(Exception):
    """Eiffel からの取得・変換に関する基底例外。"""


class EiffelUnavailable(EiffelError):
    """接続不可・タイムアウト・not ready 等、今は取得できない状態(リトライ可)。"""


class EiffelContractError(EiffelError):
    """応答の形が想定契約と違う(バグ/バージョン不整合。リトライしても直らない)。"""


@dataclass(frozen=True)
class EiffelSnapshot:
    """Eiffel から取得した1時点の迷路情報。"""

    wm: WallMap
    cols: int
    rows: int
    balls: Tuple[Tuple[int, int], ...]      # (col, row) のタプル列
    role: str
    last_update: Optional[str]              # 1号機ローカルの最終更新(ISO8601)
    peer_last_update: Optional[str]         # 2号機の最終更新。None=2号機未接続
    source: str                            # "combined" or "standalone"
    invalid_inner_edges: int               # 外周を除く walls_valid=False の内辺数
    undetected_cells: int                  # walls_valid が全 false のマス数(未検出)
    # 生の cells(検証・デバッグ用)。walls_valid 参照に使う。比較・repr からは除外。
    _cells: dict = field(default_factory=dict, repr=False, compare=False)

    @property
    def peer_connected(self) -> bool:
        return self.peer_last_update is not None

    def cell_all_valid(self, col: int, row: int) -> bool:
        """セル (col,row) の全**内辺**が walls_valid=True か(未検出=False)。

        外周辺(フィールド境界)は WallMap が常に壁扱いするので、その walls_valid が
        False でも判定に含めない(外周セルの正当なゴールを弾かないため)。範囲外セルは
        常に True。
        """
        c = self._cells.get((col, row))
        if c is None:
            return not (0 <= col < self.cols and 0 <= row < self.rows)
        return all(
            c["walls_valid"][d]
            for d in _DIR_KEYS
            if not _is_outer_edge(col, row, d, self.cols, self.rows)
        )

    def region_valid(self, cells: Iterable[Tuple[int, int]]) -> bool:
        """与えたセル群がすべて検出済み(全辺 valid)か。走行前ゲートに使う。"""
        return all(self.cell_all_valid(cx, cy) for cx, cy in cells)

    def all_valid(self) -> bool:
        """全マスが検出済みか(2号機オフラインだと東半分が未検出で False)。"""
        return self.undetected_cells == 0 and self.invalid_inner_edges == 0


def fetch_status(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout_s: float = DEFAULT_TIMEOUT_S,
) -> dict:
    """`GET /status.json` を叩いて生の dict を返す。

    接続不可・タイムアウト・非200・JSON 不正はすべて EiffelUnavailable。
    """
    url = f"http://{host}:{port}/status.json"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as resp:
            if resp.status != 200:
                raise EiffelUnavailable(f"HTTP {resp.status} from {url}")
            raw = resp.read()
    except urllib.error.URLError as e:  # timeout も URLError のサブクラス
        raise EiffelUnavailable(f"{url} へ接続できません: {e}") from e
    except OSError as e:
        raise EiffelUnavailable(f"{url} への通信に失敗: {e}") from e
    try:
        return json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as e:
        raise EiffelUnavailable(f"{url} の応答が JSON ではありません: {e}") from e


def _map_dict_from_status(status: dict) -> Tuple[dict, str]:
    """status から Liner が使う迷路 dict と source ラベルを取り出す。

    role=primary は `combined`(2台マージ済み全域)を使う。role=standalone は
    トップレベルを全域として使う。role=secondary は集約前なので使えない。
    戻り値の迷路 dict は {maze_cols, maze_rows, cells, balls,(peer_last_update)}。
    """
    if not status.get("ready", False):
        raise EiffelUnavailable(
            f"Eiffel はまだ ready ではありません (last_error={status.get('last_error')!r})"
        )
    role = status.get("role", "standalone")
    if role == "primary":
        combined = status.get("combined")
        if not isinstance(combined, dict):
            raise EiffelContractError("role=primary だが combined フィールドがありません")
        return combined, "combined"
    if role == "secondary":
        raise EiffelUnavailable(
            "role=secondary の号機には集約マップがありません。1号機(primary)へ接続してください"
        )
    # standalone: トップレベルが全域。
    return status, "standalone"


def _is_outer_edge(col: int, row: int, d: str, cols: int, rows: int) -> bool:
    """辺 (col,row,d) がフィールド外周かどうか。"""
    return (
        (d == "W" and col == 0)
        or (d == "E" and col == cols - 1)
        or (d == "S" and row == 0)
        or (d == "N" and row == rows - 1)
    )


def wallmap_from_map(
    map_dict: dict, valid_policy: str = "conservative"
) -> Tuple[WallMap, dict]:
    """迷路 dict(combined 等)から WallMap を構築し、統計 dict を併せて返す。

    valid_policy:
      "strict"       … walls[d] and walls_valid[d] の辺だけ壁を立てる(未知は開放)。
      "conservative" … 上記に加え、内辺で walls_valid[d]=False の辺も壁を立てて封鎖する
                       (未検出マスへ経路が伸びない=安全側。既定)。
    どちらでも外周辺の invalid は無視(WallMap が外周を常に壁扱いするため)。

    統計 dict: {invalid_inner_edges, undetected_cells}。
    """
    if valid_policy not in ("strict", "conservative"):
        raise ValueError(f"未知の valid_policy: {valid_policy!r}")
    try:
        cols = int(map_dict["maze_cols"])
        rows = int(map_dict["maze_rows"])
        cells = map_dict["cells"]
    except (KeyError, TypeError, ValueError) as e:
        raise EiffelContractError(f"迷路 dict の必須フィールドが不正です: {e}") from e

    wm = WallMap(cols, rows)
    invalid_inner_edges = 0
    undetected_cells = 0
    for c in cells:
        try:
            col, row = int(c["col"]), int(c["row"])
            walls, valid = c["walls"], c["walls_valid"]
        except (KeyError, TypeError, ValueError) as e:
            raise EiffelContractError(f"cell の形が不正です: {c!r} ({e})") from e
        if all(not valid[d] for d in _DIR_KEYS):
            undetected_cells += 1
        for d in _DIR_KEYS:
            is_valid = bool(valid[d])
            is_wall = bool(walls[d])
            outer = _is_outer_edge(col, row, d, cols, rows)
            if not is_valid and not outer:
                invalid_inner_edges += 1
            if is_valid:
                if is_wall:
                    wm.add_wall(col, row, Direction[d])
            elif valid_policy == "conservative" and not outer:
                # 未知の内辺は封鎖(安全側)。
                wm.add_wall(col, row, Direction[d])
    stats = {
        "invalid_inner_edges": invalid_inner_edges,
        "undetected_cells": undetected_cells,
    }
    return wm, stats


def snapshot_from_status(
    status: dict, valid_policy: str = "conservative"
) -> EiffelSnapshot:
    """status(生 dict)から EiffelSnapshot を構築する(HTTP 非依存、テスト容易)。"""
    map_dict, source = _map_dict_from_status(status)
    wm, stats = wallmap_from_map(map_dict, valid_policy=valid_policy)
    cols = int(map_dict["maze_cols"])
    rows = int(map_dict["maze_rows"])
    cell_index = {}
    for c in map_dict["cells"]:
        cell_index[(int(c["col"]), int(c["row"]))] = c
    balls = tuple(
        (int(b["col"]), int(b["row"])) for b in map_dict.get("balls", [])
    )
    return EiffelSnapshot(
        wm=wm,
        cols=cols,
        rows=rows,
        balls=balls,
        role=status.get("role", "standalone"),
        last_update=status.get("last_update"),
        peer_last_update=map_dict.get("peer_last_update"),
        source=source,
        invalid_inner_edges=stats["invalid_inner_edges"],
        undetected_cells=stats["undetected_cells"],
        _cells=cell_index,
    )


def fetch_snapshot(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    valid_policy: str = "conservative",
) -> EiffelSnapshot:
    """HTTP 取得 + 変換の合成。接続不可/not ready は EiffelUnavailable。"""
    status = fetch_status(host, port, timeout_s)
    return snapshot_from_status(status, valid_policy=valid_policy)
