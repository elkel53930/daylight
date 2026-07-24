"""マイクロマウス設定。

競技規則に関わる値(迷路サイズ・セル寸法・ゴール領域)や機体調整値
(速度・しきい値)をハードコードせず、YAML で上書きできるようにする。
YAML が無ければ既定値で動く。
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Optional

from maze import GoalRegion

DEFAULT_CONFIG_PATH = Path(__file__).parent / "config" / "micromouse.yaml"


@dataclass
class MicromouseConfig:
    # --- 通信 ---
    port: str = "/dev/ttyUSB0"
    baud: int = 3000000
    command_timeout_s: float = 10.0   # 走行コマンドの DONE 待ちタイムアウト

    # --- 迷路(競技規則) ---
    maze_size: int = 16
    cell_size_mm: float = 180.0
    goal_x_min: int = 7
    goal_x_max: int = 8
    goal_y_min: int = 7
    goal_y_max: int = 8

    # --- 走行(初期実装は低速優先。高速化は安定後) ---
    explore_speed_mmps: float = 300.0
    explore_accel_mmps2: float = 1000.0
    speed_run_speed_mmps: float = 400.0
    speed_run_accel_mmps2: float = 1200.0
    start_offset_mm: float = 90.0     # スタート時、セル中心→最初の判断点までの距離

    # --- 壁判定しきい値(hw_test.py walls で校正する) ---
    wall_left_threshold: int = 100
    wall_right_threshold: int = 100
    wall_front_threshold: int = 50

    # --- 安全 ---
    battery_min_v: float = 6.5
    max_exploration_steps: int = 1000
    sensor_retry: int = 3
    sensor_max_consecutive_failures: int = 5

    # --- カメラによる位置補正(camera_correction.py) ---
    # 前壁がある判断点で、前回の補正からこのセル数以上移動していたら
    # カメラでヨー角・前進距離を補正し、ジャイロも再キャリブレーションする。
    # 実機(MobileBase)でのみ有効(SimMobileBaseにはjog_turn等が無いため
    # 自動的に無効になる、state_machine.py参照)。
    camera_correction_enabled: bool = False
    camera_correction_interval_cells: int = 10

    # --- その他 ---
    log_dir: str = str(Path(__file__).parent / "logs")

    def goal_region(self) -> GoalRegion:
        return GoalRegion(
            x_min=self.goal_x_min,
            x_max=self.goal_x_max,
            y_min=self.goal_y_min,
            y_max=self.goal_y_max,
        )

    @property
    def half_cell_mm(self) -> float:
        return self.cell_size_mm / 2.0

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "MicromouseConfig":
        """YAML から読み込む。ファイルが無ければ既定値。

        未知のキーは無視せずエラーにする(タイポの検出のため)。
        """
        config = cls()
        target = path or DEFAULT_CONFIG_PATH
        if not Path(target).exists():
            if path is not None:
                raise FileNotFoundError(f"config not found: {path}")
            return config

        import yaml

        with open(target, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f"config root must be a mapping: {target}")

        valid = {f.name: f.type for f in fields(cls)}
        for key, value in data.items():
            if key not in valid:
                raise ValueError(f"unknown config key: {key} (in {target})")
            setattr(config, key, value)
        return config
