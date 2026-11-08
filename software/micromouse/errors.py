"""共有例外定義。

mobile_base.py は pyserial に依存するため、シミュレーションやテストが
例外型のためだけに pyserial を要求しないよう、ここに分離している。
"""

from __future__ import annotations


class MobileBaseError(Exception):
    """通信エラー・タイムアウトなど、安全停止すべき異常。"""


class AbortRequested(Exception):
    """ユーザー操作による中断要求。発生時点でモータは停止済み。"""
