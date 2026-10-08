"""LiteGrip SDK - 异常定义"""

from __future__ import annotations


class LiteGripError(Exception):
    """LiteGrip 基础异常"""

    def __init__(self, message: str, error_code: int | None = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code

    def __str__(self) -> str:
        if self.error_code is not None:
            return f"{self.message} [0x{self.error_code:04X}]"
        return self.message


class CommError(LiteGripError):
    """通信错误（CAN 总线读写失败）"""
    pass


class ConnectError(LiteGripError):
    """连接错误（CAN 接口不可用、电机无应答等）"""
    pass


class CommandError(LiteGripError):
    """指令错误（电机拒绝执行或参数越界）"""
    pass


class CANTimeoutError(LiteGripError):
    """超时错误（CAN 总线无应答）"""
    pass


class HardwareError(LiteGripError):
    """硬件故障（欠压/过流/过温等达妙电机故障码）"""
    pass


class NotInitializedError(LiteGripError):
    """未初始化错误（未连接/未使能时调用了需要使能的操作）"""
    pass
