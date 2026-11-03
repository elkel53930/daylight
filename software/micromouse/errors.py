"""共有例外定義。

=== 概要 ===
mobile_base.py は pyserial に依存するため、シミュレーションやテストが
例外型のためだけに pyserial を要求しないよう、ここに分離している。

=== 例外クラス ===
MobileBaseError: 通信エラー・タイムアウトなど、安全停止すべき異常
  - センサ読み取り失敗
  - コマンド応答タイムアウト
  - シリアル通信エラー
  
AbortRequested: ユーザー操作による中断要求
  - OLED の L ボタン押下
  - Ctrl+C による中断
  - 発生時点でモータは停止済み
"""

from __future__ import annotations


class MobileBaseError(Exception):
    """通信エラー・タイムアウトなど、安全停止すべき異常。"""


class AbortRequested(Exception):
    """ユーザー操作による中断要求。発生時点でモータは停止済み。"""
