"""diag_sim.py — 45°スラローム+斜め直進パターンの壁クリアランス検証(シミュレーション)。

path_controller.cpp の公式(内部フレーム)を忠実に再現し、軌跡をワールド座標へ
変換して、開発迷路の全壁セグメントとの最小距離を計算する。モータは動かさない。

使い方:
    ../venv/bin/python3 diag_sim.py
"""
from __future__ import annotations

import math

CELL = 180.0


def wall_segments():
    """開発迷路の内壁を世界座標の線分(x1,y1,x2,y2)で返す(原点=南西隅)。"""
    # (cx, cy, 方向, x1, y1, x2, y2)
    walls = [
        ((0, 3), "S", 0.0, 540.0, 180.0, 540.0),
        ((2, 3), "S", 360.0, 540.0, 540.0, 540.0),
        ((2, 2), "S", 360.0, 360.0, 540.0, 360.0),
        ((1, 1), "S", 180.0, 180.0, 360.0, 180.0),
        ((1, 1), "W", 180.0, 180.0, 180.0, 360.0),
        ((1, 0), "W", 180.0, 0.0, 180.0, 180.0),
        ((2, 1), "E", 540.0, 180.0, 540.0, 360.0),
    ]
    return [(x1, y1, x2, y2) for _, _, x1, y1, x2, y2 in walls]


def dist_point_segment(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px - x1, py - y1
    L2 = vx * vx + vy * vy
    if L2 == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))


def trace(pat, sx0, sy0, theta0, sample_step=3.0, slalom_step_deg=2.0):
    """区間列から軌跡ポリライン(世界座標)を返す。

    pat: [("STRAIGHT", length_mm) | ("SLALOM", dir, R, angle_deg), ...]
    内部フレーム(x=進行方向開始, y=左)は path_controller の式そのまま。
    """
    pts = [(sx0, sy0)]
    cx, cy = math.cos(theta0), math.sin(theta0)
    sx, sy = -math.sin(theta0), math.cos(theta0)  # 左方向ベクトル
    ix, iy = 0.0, 0.0
    head = 0.0  # 内部 heading(進行方向=+x が0)

    def push(ix, iy):
        pts.append((sx0 + cx * ix + sx * iy, sy0 + cy * ix + sy * iy))

    for item in pat:
        if item[0] == "STRAIGHT":
            length = item[1]
            steps = max(2, int(length / sample_step))
            for k in range(1, steps + 1):
                p = length * k / steps
                push(ix + p * math.cos(head), iy + p * math.sin(head))
            ix += length * math.cos(head)
            iy += length * math.sin(head)
        else:
            dirv, R, ang_deg = item[1], item[2], item[3]
            phi_end = math.radians(ang_deg)
            steps = max(8, int(phi_end / math.radians(slalom_step_deg)))
            for k in range(1, steps + 1):
                phi = phi_end * k / steps
                th = head + dirv * phi
                ix2 = ix + dirv * R * (math.sin(th) - math.sin(head))
                iy2 = iy + dirv * R * (math.cos(head) - math.cos(th))
                push(ix2, iy2)
            head += dirv * phi_end
            ix, iy = ix2, iy2
    return pts


def min_clearance(pts, walls):
    return min(min(dist_point_segment(px, py, *w) for w in walls) for px, py in pts)


def main():
    walls = wall_segments()
    theta0 = math.radians(90.0)  # 北
    for start in ((90.0, 270.0), (90.0, 360.0)):
        for L in (240.0, 280.0, 320.0, 344.5):
            pat = [
                ("STRAIGHT", 90.0),
                ("SLALOM", -1, 90.0, 45.0),
                ("STRAIGHT", L),
            ]
            pts = trace(pat, start[0], start[1], theta0)
            d = min_clearance(pts, walls)
            end = pts[-1]
            print(f"start=({start[0]:.0f},{start[1]:.0f}) L={L:6.1f}mm: "
                  f"終点=({end[0]:7.1f},{end[1]:7.1f}) min={d:5.1f}mm "
                  f"({'OK' if d >= 50 else 'NG(50mm基準)'})")


if __name__ == "__main__":
    main()
