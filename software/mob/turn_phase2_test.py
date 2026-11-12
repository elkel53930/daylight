#!/usr/bin/env python3
"""Phase 2: 旋回振動の根本原因特定（TURNコマンド版）.

TURNコマンドを使い、DONEを待たずにテレメトリをキャプチャする。
各テスト前に角度リセット（RANG）を行い、ブザーで警告する。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "micromouse"))
sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))

import serial
from config import MicromouseConfig

try:
    from ui_client import UIClient
    UI_AVAILABLE = True
except ImportError:
    UI_AVAILABLE = False
    UIClient = None

# テレメトリの正規表現（新しいファームウェア形式）
A_TELEMETRY_RE = re.compile(r"^#A,([-\d.]+),([-\d.]+),([-\d.]+),([-\d]+)")
P_TELEMETRY_RE = re.compile(r"^#P,([-\d.]+),([-\d.]+),([-\d.]+),([-\d]+)")


def warn_before_move(ui: UIClient = None) -> None:
    """動作前にブザーで警告し、2秒間待機する."""
    print("⚠️  動作開始: ブザーを鳴らします")
    if ui is not None:
        try:
            ui.play("ceg")
        except Exception as e:
            print(f"  ブザー鳴音失敗: {e}")
    else:
        print("  (UIサーバー利用不可)")
    print("  2秒後に旋回を開始します...")
    time.sleep(2.0)
    print("  開始!")


def set_param(ser: serial.Serial, name: str, value: float) -> bool:
    """PSETコマンドでパラメータを設定."""
    ser.write(f"PSET,{name},{value}\n".encode("ascii"))
    time.sleep(0.1)
    # 応答を待つ（PSETは "#PSET name=value" を返す）
    ser.timeout = 0.5
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if f"#PSET {name}=" in line:
            return True
        if line:
            print(f"  Response: {line}")
    return False


def capture_turn(ser: serial.Serial, angle_rad: float, duration_s: float = 5.0) -> list[dict]:
    """TURNコマンドで旋回し、テレメトリをキャプチャ（DONEを待たない）."""
    ser.reset_input_buffer()
    
    # 角度リセット
    ser.write(b"RANG\n")
    time.sleep(0.3)
    ser.readline()  # 応答を読み捨
    
    start_time = time.monotonic()
    ser.write(f"TURN,{angle_rad:.6f}\n".encode("ascii"))
    
    data_points = []
    capture_end = start_time + duration_s
    ser.timeout = 0.01
    
    while time.monotonic() < capture_end:
        try:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            
            t_ms = (time.monotonic() - start_time) * 1000
            
            # #Aテレメトリ（旋回制御）
            m = A_TELEMETRY_RE.match(line)
            if m:
                data_points.append({
                    "time_ms": t_ms,
                    "target_angle_rad": float(m.group(1)),
                    "actual_angle_rad": float(m.group(2)),
                    "omega_target_radps": float(m.group(3)),
                    "duty_diff": int(m.group(4)),
                })
        except Exception:
            pass
    
    return data_points


def analyze(data: list[dict], target_angle_rad: float) -> dict:
    """テレメトリデータを分析."""
    if not data:
        return {"error": "no data", "final_error_deg": 0, "max_error_deg": 0, "overshoot_deg": 0, "crossings": 0, "samples": 0}
    
    final_angle = data[-1]["actual_angle_rad"]
    angle_error = final_angle - target_angle_rad
    
    # 減速開始後のデータ（目標角速度が小さい）
    settled_data = [d for d in data if abs(d["omega_target_radps"]) < 1.0]
    if settled_data:
        errors = [d["actual_angle_rad"] - target_angle_rad for d in settled_data]
        max_error = max(abs(e) for e in errors)
        
        # オーバーシュート検出
        max_angle = max(d["actual_angle_rad"] for d in data)
        min_angle = min(d["actual_angle_rad"] for d in data)
        if target_angle_rad > 0:
            overshoot = max_angle - target_angle_rad
        else:
            overshoot = target_angle_rad - min_angle
        
        # 交叉回数
        crossings = 0
        for i in range(1, len(settled_data)):
            prev_err = settled_data[i-1]["actual_angle_rad"] - target_angle_rad
            curr_err = settled_data[i]["actual_angle_rad"] - target_angle_rad
            if prev_err * curr_err < 0:
                crossings += 1
        
        return {
            "final_error_deg": math.degrees(angle_error),
            "max_error_deg": math.degrees(max_error),
            "overshoot_deg": math.degrees(overshoot),
            "crossings": crossings,
            "samples": len(settled_data),
        }
    return {
        "final_error_deg": math.degrees(angle_error),
        "max_error_deg": 0,
        "overshoot_deg": 0,
        "crossings": 0,
        "samples": 0,
    }


def run_test(ser: serial.Serial, name: str, params: dict, angle_rad: float = math.pi/2, ui: UIClient = None) -> dict:
    """1つのパラメータ設定でテスト実行."""
    print(f"\n{'='*60}")
    print(f"{name}")
    print('='*60)
    
    # パラメータ設定
    for pname, pvalue in params.items():
        print(f"  {pname} = {pvalue}")
        if not set_param(ser, pname, pvalue):
            print(f"  WARNING: Failed to set {pname}")
    
    time.sleep(0.5)
    
    # 安全確認: ブザーを鳴らして2秒待機
    warn_before_move(ui)
    
    data = capture_turn(ser, angle_rad)
    result = analyze(data, angle_rad)
    
    # 詳細ログ表示
    if data:
        print("\n--- 旋回ログ（抜粋）---")
        print(f"{'時刻[ms]':>10} {'角度[deg]':>10} {'目標角速度':>12} {'duty_diff':>10}")
        for d in data[::max(1, len(data)//10)]:
            angle_deg = d['actual_angle_rad'] * 180 / math.pi
            print(f"{d['time_ms']:>10.1f} {angle_deg:>10.1f} {d['omega_target_radps']:>12.2f} {d['duty_diff']:>10}")
        
        final = data[-1]
        print(f"\n--- 最終状態 ---")
        print(f"  最終角度: {final['actual_angle_rad']*180/math.pi:.2f} deg")
        print(f"  目標角度: {angle_rad*180/math.pi:.2f} deg")
    
    print(f"\n=== 結果 ===")
    print(f"  最終誤差: {result['final_error_deg']:+.2f}°")
    print(f"  最大誤差: {result['max_error_deg']:.2f}°")
    print(f"  オーバーシュート: {result['overshoot_deg']:.2f}°")
    print(f"  目標値交叉回数: {result['crossings']}")
    
    # 異常検出
    if result['final_error_deg'] > 20:
        print("\n⚠️  WARNING: 最終誤差が20度を超えています！")
    if result['overshoot_deg'] > 30:
        print("\n⚠️  WARNING: オーバーシュートが30度を超えています！")
    
    return result


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=3000000)
    args = ap.parse_args()
    
    # UIクライアント初期化（ブザー用）
    ui = None
    if UI_AVAILABLE:
        try:
            ui = UIClient()
            ui.connect()
            print("UIサーバーに接続しました（ブザー利用可能）")
        except Exception as e:
            print(f"UIサーバー接続失敗: {e}")
            ui = None
    else:
        print("UIクライアント利用不可")
    
    # シリアル接続
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    time.sleep(1)
    
    try:
        results = []
        angle_rad = math.pi / 2  # 左90度旋回
        
        # Test 1: ベースライン（現状）
        results.append(("Baseline", run_test(ser, "Test 1: Baseline", {}, angle_rad, ui)))
        
        # Test 2: 旋回速度を下げる
        results.append(("MaxRadps/2", run_test(ser, "Test 2: pivot_max_radps = 3.9", {"pivot_max_radps": 3.9}, angle_rad, ui)))
        
        # Test 3: 旋回速度をさらに下げる
        results.append(("MaxRadps/4", run_test(ser, "Test 3: pivot_max_radps = 2.0", {"pivot_max_radps": 2.0}, angle_rad, ui)))
        
        # Test 4: 加減速を緩やかに
        results.append(("Accel/2", run_test(ser, "Test 4: pivot_accel = 10", {"pivot_accel": 10}, angle_rad, ui)))
        
        # Test 5: 加減速をさらに緩やかに
        results.append(("Accel/4", run_test(ser, "Test 5: pivot_accel = 5", {"pivot_accel": 5}, angle_rad, ui)))
        
        # Test 6: Pゲインを下げる
        results.append(("P/2", run_test(ser, "Test 6: pivot_kp = 200", {"pivot_kp": 200}, angle_rad, ui)))
        
        # Test 7: Pゲインをさらに下げる
        results.append(("P/4", run_test(ser, "Test 7: pivot_kp = 100", {"pivot_kp": 100}, angle_rad, ui)))
        
        # Test 8: Iゲインを0に
        results.append(("I=0", run_test(ser, "Test 8: pivot_ki = 0", {"pivot_ki": 0}, angle_rad, ui)))
        
        # Test 9: Dゲインを下げる
        results.append(("D/2", run_test(ser, "Test 9: pivot_kd = 25", {"pivot_kd": 25}, angle_rad, ui)))
        
        # Test 10: 旋回速度と加減速の両方を下げる
        results.append(("Slow+Gentle", run_test(ser, "Test 10: max_radps=3.9, accel=10", 
                                               {"pivot_max_radps": 3.9, "pivot_accel": 10}, angle_rad, ui)))
        
        # 結果サマリー
        print("\n" + "="*60)
        print("Phase 2 結果サマリー")
        print("="*60)
        print(f"{'Test':<15} {'FinalErr':>10} {'MaxErr':>10} {'Overshoot':>10} {'Cross':>8}")
        print("-"*60)
        for name, r in results:
            print(f"{name:<15} {r['final_error_deg']:>+9.2f}° {r['max_error_deg']:>9.2f}° "
                  f"{r['overshoot_deg']:>9.2f}° {r['crossings']:>7}")
        
        return 0
        
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 0
    finally:
        ser.write(b"MOT,0,0\n")
        ser.close()
        if ui is not None:
            try:
                ui.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
