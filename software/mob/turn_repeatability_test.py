#!/usr/bin/env python3
"""旋回再現性テスト: 90度右旋回を10回繰り返す.

目的:
- 旋回後の最終的な誤差のバラツキ（再現性）を確認
- ジャイロセンサの計測値と実際の物理角度の乖離を確認
"""

import sys
sys.path.insert(0, 'software/ui')

import serial
import time
import math
import re
import statistics
from datetime import datetime
from pathlib import Path

try:
    from ui_client import UIClient
    ui = UIClient()
    ui.connect()
    print('UIサーバーに接続しました（ブザー利用可能）')
except Exception as e:
    print(f'UI利用不可: {e}')
    ui = None

# シリアル接続
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=3000000, timeout=0.1)
time.sleep(1)

# ログファイル準備
stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
log_file = Path(f'software/mob/turn_repeatability_test_{stamp}.csv')
with open(log_file, 'w') as f:
    f.write('trial,final_angle_deg,error_deg,max_angle_deg,overshoot_deg,crossings\n')

# テレメトリ正規表現
A_TELEMETRY_RE = re.compile(r'^#A,([-\d.]+),([-\d.]+),([-\d.]+),([-\d]+)')
SEN_RE = re.compile(r'^SEN,')


def read_angle(ser, timeout_s=1.0):
    """SEN を要求して現在角度[rad]を取得。取得できなければ None。"""
    ser.reset_input_buffer()
    ser.write(b'SEN\n')
    deadline = time.monotonic() + timeout_s
    ser.timeout = 0.05
    while time.monotonic() < deadline:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if SEN_RE.match(line):
            parts = line.split(',')
            try:
                return float(parts[10])
            except (IndexError, ValueError):
                return None
    return None


def safe_reset_angle(ser):
    """TURN保持を抜けてからRANGし、DONEまで確実に読み切る。

    ⚠️ CLAUDE.md: TURN保持中(place_controller動作中)にRANGを出すと、
    turn_goalは旧フレームのまま残り angle_error が巨大化して暴走する。
    必ず MOT,0,0 で PLACE_HOLD を抜けてから RANG すること。
    """
    # 1. TURN保持/位置保持を抜ける(IDLEへ)
    ser.write(b'MOT,0,0\n')
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 2. RANG を送り、DONE まで読み切る(途中の #P/#A テレメトリは捨てる)
    ser.write(b'RANG\n')
    deadline = time.monotonic() + 2.0
    ser.timeout = 0.05
    got_done = False
    while time.monotonic() < deadline:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line == 'DONE':
            got_done = True
            break
    if not got_done:
        raise RuntimeError('RANG の DONE を受信できませんでした')

    # 3. 角度が0付近であることを検証(旧フレーム残留の検出)
    ang = read_angle(ser)
    if ang is None:
        raise RuntimeError('リセット後の角度を取得できませんでした')
    if abs(ang) > math.radians(2.0):
        raise RuntimeError(f'角度リセット失敗: {math.degrees(ang):+.2f}deg (0付近でない)')
    return ang


# テスト結果
results = []

# 90度右旋回を10回繰り返す
NUM_TRIALS = 10
target_angle_rad = -math.pi / 2  # 右90度
unsafe_abort = False

for trial in range(1, NUM_TRIALS + 1):
    print(f'\n{"="*60}')
    print(f'Trial {trial}/{NUM_TRIALS}')
    print('='*60)
    
    # ブザーで警告
    if ui:
        print('⚠️  動作開始: ブザーを鳴らします')
        try:
            ui.play('ceg')
        except:
            pass
        print('  2秒後に旋回を開始します...')
        time.sleep(2.0)
    else:
        print('2秒待機...')
        time.sleep(2.0)
    
    print('  開始!')
    
    # 角度リセット(MOT,0,0でTURN保持を抜けてからRANG、DONE待ち、角度0検証)
    try:
        ang0 = safe_reset_angle(ser)
        print(f'  角度リセット後: {math.degrees(ang0):+.2f} deg')
    except RuntimeError as e:
        print(f'  ⚠️  安全のためTrial {trial}を中止: {e}')
        ser.write(b'MOT,0,0\n')
        break
    
    # 旋回実行
    start_time = time.monotonic()
    ser.write(f'TURN,{target_angle_rad:.6f}\n'.encode('ascii'))
    
    # テレメトリキャプチャ
    data_points = []
    capture_end = start_time + 5.0
    
    while time.monotonic() < capture_end:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
            
            t_ms = (time.monotonic() - start_time) * 1000
            m = A_TELEMETRY_RE.match(line)
            if m:
                actual = float(m.group(2))
                data_points.append({
                    'time_ms': t_ms,
                    'target': float(m.group(1)),
                    'actual': actual,
                    'omega': float(m.group(3)),
                    'duty': int(m.group(4)),
                })
                # 安全網: 開始0.5s以降で目標から45度以上外れたら即停止
                if t_ms > 500 and abs(actual - target_angle_rad) > math.radians(45.0):
                    print(f'  ⚠️  暴走検出: 角度={math.degrees(actual):+.1f}deg '
                          f'(目標{math.degrees(target_angle_rad):+.1f}deg) → 即停止')
                    ser.write(b'MOT,0,0\n')
                    time.sleep(0.5)
                    ser.reset_input_buffer()
                    data_points = []
                    unsafe_abort = True
                    break
        except Exception:
            pass
    
    # 結果計算
    if data_points:
        final = data_points[-1]
        final_angle = final['actual']
        error = final_angle - target_angle_rad
        
        # オーバーシュート計算
        min_angle = min(d['actual'] for d in data_points)
        overshoot = target_angle_rad - min_angle
        
        # 交叉回数
        crossings = 0
        settled_data = [d for d in data_points if abs(d['omega']) < 1.0]
        for i in range(1, len(settled_data)):
            prev_err = settled_data[i-1]['actual'] - target_angle_rad
            curr_err = settled_data[i]['actual'] - target_angle_rad
            if prev_err * curr_err < 0:
                crossings += 1
        
        result = {
            'trial': trial,
            'final_angle_deg': final_angle * 180 / math.pi,
            'error_deg': error * 180 / math.pi,
            'max_angle_deg': min_angle * 180 / math.pi,
            'overshoot_deg': overshoot * 180 / math.pi,
            'crossings': crossings,
        }
        results.append(result)
        
        # CSVに書き込み
        with open(log_file, 'a') as f:
            f.write(f"{trial},{result['final_angle_deg']:.2f},{result['error_deg']:.2f},"
                    f"{result['max_angle_deg']:.2f},{result['overshoot_deg']:.2f},{result['crossings']}\n")
        
        # 表示
        print(f'\n--- Trial {trial} 結果 ---')
        print(f'  最終角度: {result["final_angle_deg"]:.2f} deg')
        print(f'  目標角度: -90.00 deg')
        print(f'  最終誤差: {result["error_deg"]:+.2f} deg')
        print(f'  最大角度: {result["max_angle_deg"]:.2f} deg')
        print(f'  オーバーシュート: {result["overshoot_deg"]:.2f} deg')
        print(f'  交叉回数: {result["crossings"]}')
    else:
        print(f'\n--- Trial {trial}: データ取得失敗 ---')
    
    if unsafe_abort:
        print('\n⚠️  暴走を検出したためテストを中断します')
        ser.write(b'MOT,0,0\n')
        break

ser.close()

# 統計計算
if results:
    errors = [r['error_deg'] for r in results]
    overshoots = [r['overshoot_deg'] for r in results]
    
    print('\n' + '='*60)
    print('統計サマリー')
    print('='*60)
    
    print(f'\n最終誤差 [deg]:')
    print(f'  平均: {statistics.mean(errors):+.2f}')
    print(f'  標準偏差: {statistics.stdev(errors):.2f}')
    print(f'  最小: {min(errors):+.2f}')
    print(f'  最大: {max(errors):+.2f}')
    print(f'  範囲: {max(errors) - min(errors):.2f}')
    
    print(f'\nオーバーシュート [deg]:')
    print(f'  平均: {statistics.mean(overshoots):.2f}')
    print(f'  標準偏差: {statistics.stdev(overshoots):.2f}')
    print(f'  最小: {min(overshoots):.2f}')
    print(f'  最大: {max(overshoots):.2f}')
    
    print(f'\n全Trial結果:')
    print(f'{"Trial":>6} {"最終角度":>10} {"誤差":>8} {"オーバーシュート":>12} {"交叉":>6}')
    print('-'*50)
    for r in results:
        print(f'{r["trial"]:>6} {r["final_angle_deg"]:>10.2f} {r["error_deg"]:>+8.2f} '
              f'{r["overshoot_deg"]:>12.2f} {r["crossings"]:>6}')

print(f'\nログファイル: {log_file}')

if ui:
    try:
        ui.disconnect()
    except:
        pass
