"""LiteGrip 动作层 —— open / close / grasp / zero / enable / disable。

高层 ``LiteGrip`` 的 ``open()`` / ``close()`` / ``grasp()`` 等方法直接转发到
这里的 :class:`GripperActions`。单独成模块是为了让「怎么动」这套逻辑只写一遍：
ROS 2 桥接、RPC 服务、产品代码都能直接调 :attr:`LiteGrip.actions`，而不用各自
重写一遍斜坡和堵转判据。

为什么不用 ``control_mit_stream()``
-----------------------------------
``control_mit_stream`` 在整段时长里反复下发**同一个** ``q_target``：伺服几十
毫秒就贴上去，剩下时间空转 —— 慢速时表现为一顿一停。这里按固定帧间隔推进一条
线性斜坡，并给 ``dq_target`` 速度前馈，所以是匀速连续运动。

``goto`` 一族（``goto`` / ``goto_rad`` / ``move_to`` / ``home``）和
``move_at_speed`` 一族原来各自就是那个 ``control_mit_stream`` 式的硬阶跃；现在
它们都转发到这里（:meth:`GripperActions.ramp_to`），「怎么动」只剩这一份实现。

指令领先实测位置的部分由领先上限封顶，分两档：行进段用
:attr:`MotionConfig.max_lead_mm`（大领先量破静摩擦），距限位
:attr:`MotionConfig.press_zone_mm` 之内切到
:attr:`MotionConfig.stop_lead_mm`（压紧段轻压）。不封顶的话，被挡住时误差会
一直累积、力矩顶到危险值；封顶后静摩擦靠满额力矩破，力矩却始终有界（约
``kp × lead_cap_rad``）。也不能改成「相对实测加一块」——那样一旦夹住，指令跟
着实测冻结，误差永远涨不上去，会误判堵转。

``open()`` / ``close()`` 的目标是**越过**标定限位一点（
:attr:`MotionConfig.press_overshoot`），好让运动靠堵转结束在限位上；但**下发的
指令**由 :func:`clamp_to_travel` 封顶在标定限位，夹爪不可能被驱动到标定行程
之外。「目标越位、指令不越位」是一次移动的两面：目标决定什么时候判堵转，指令
决定夹爪能走到哪。``grasp()`` 的闭合段瞄的也是**限位本身**（走得慢、带着设定力
的预算）：空载时夹爪在限位上停下，夹到工件时在工件上停下，两种都是停着的状态，
正是保力段可以接手的前提（见 :meth:`GripperActions.grasp`）。

**带设定力的接近段另有一条预算**（``grasp`` 的闭合段，见
:func:`force_approach_terms`）：上面那条 ``kp × 领先上限`` 对默认的 4 mm 上限、
配置默认的 ``rad_to_mm`` 是 5.30 Nm ≈ 53 N，与这一次夹取设定的力无关，于是
「夹住工件」和「先拿 53 N 撞一下」是同一件事。所以只要这一段带着设定力，它的力矩预算（设定值 ×
:attr:`MotionConfig.press_safety`）就分给这一帧的速度、阻尼和领先，撞上工件
时压出的力矩不超过设定值。代价是领先跟着设定力变窄，而设定力低到一帧的位移
自己能占满预算时，接近**速度**也由设定力定。

堵转判据是软件侧的位置增量判据（电机本身没有堵转保护）：每
:attr:`MotionConfig.sample_interval` 采一次位置，连续
:attr:`MotionConfig.stall_cycles` 次采样的**窗口净位移**小于阈值即判堵转。
阈值 = ``max(stall_delta, stall_ratio × 窗口内本该走的距离)``。不看单点，因为
闭合侧有约 0.010 rad 的机械死区，慢速粘滑时单点忽大忽小。斜坡走完的保压段
不判（那时夹爪本来就该不动）。

保力段（``grasp`` 的第二段）反过来：只下发前馈力矩，不给位置/速度增益。
力控要的是力，带增益就会随夹爪的位移衰减，详见
:meth:`GripperActions._hold_force`。

纯力矩帧里**没有位置误差**，所以上面那条「指令领先上限」的位置护栏对它无能为
力 —— 它有自己的那条：越出闭合侧标定限位超过
:attr:`MotionConfig.hold_runaway_rad` 即撤力中止
（:func:`closing_travel_excursion_rad`）。两段护栏守的是两件事：那一端封顶的是
**指令**，这一端止住的是**失控**。

本模块**不 print**：进度通过 ``progress`` 回调交给调用方（CLI 打印、ROS 节点
记日志、RPC 服务转发都行）。
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Optional, Tuple

from .constants import UnitConversion
from .exceptions import (
    CommandError,
    CommError,
    LiteGripError,
    NotInitializedError,
)
from .models import CalibrationData, GripperConfig, GripperState

if TYPE_CHECKING:                                    # 避免运行时循环 import
    from .gripper import LiteGrip

log = logging.getLogger("litegrip")


# ═══════════════════════════════════════════════════════════════════════════
# 可调量
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class MotionConfig:
    """open / close / grasp / zero / enable 的全部可调量。

    默认值即真机上调好的那组（50 mm/s、5 ms 帧、4 mm 领先上限等），
    不改就能用。速度单位一律 mm/s（开口量），内部按
    ``GripperConfig.rad_to_mm`` 换算成 rad。

    ``sleep_fn`` / ``monotonic_fn`` 是给测试和仿真留的缝：引擎内部一律走
    这两个，不直接调 ``time.*``。测试里传 ``sleep_fn=lambda _: None`` 就能让
    整条斜坡瞬间跑完，不必 monkeypatch ``time.sleep``。
    """

    # ── 运动 ───────────────────────────────────────────────────────────
    speed_mm_s: float = 50.0            # open/close 速度
    grasp_speed_mm_s: float = 50.0      # grasp 闭合段速度
    margin: float = 0.05                # limit_target 在限位内侧留的余量比例
    frame_interval: float = 0.005       # 200 Hz 斜坡帧间隔 s
    sample_interval: float = 0.05       # 20 Hz 堵转采样间隔 s
    settle_s: float = 0.3               # 斜坡后原地保目标时长 s（不判堵转）
    reach_tol: float = 0.02             # 到位容差 rad（闭合侧死区 0.0103）

    # ── open/close 顶限位 ──────────────────────────────────────────────
    # 这几个数只影响**目标的排程**和接近段的领先：真正下发的指令被
    # :func:`clamp_to_travel` 封顶在标定限位，所以越位量调多大都不会把夹爪送
    # 到标定行程之外。
    press_overshoot: float = 0.05       # 目标越过标定限位的行程比例（不是指令）
    press_zone_mm: float = 2.0          # 距限位这么近就切到 stop_lead_mm
    stop_lead_mm: float = 0.7           # 压紧段领先上限 mm（≈ kp × 上限）
    # 停稳位置距限位多近算「顶在限位上」rad。这个数必须容得下「记录限位比
    # 压紧能到的地方深一点」：``zero()`` 用 calib_kp × calib_step_rad（≈1.0 Nm）
    # 探测，而 close() 按自己的刚度压，两者用力不同。台架实测缺口 0.92–1.27 mm
    # （@57.55 mm/rad 即 0.016–0.022 rad），0.02 只差 0.001 rad 就够不着，于是
    # 照文档顺序跑 stroke_test 第一遍就 exit 1。0.03 留了约 36% 余量，仍只占
    # 1.41 rad 行程的 2.1%。
    stop_tol: float = 0.03              # 停稳位置距限位多近算「顶在限位上」rad

    # ── 堵转判据 ───────────────────────────────────────────────────────
    stall_cycles: int = 5               # 窗口采样点数
    stall_ratio: float = 0.2            # 窗口净位移 / 本该走的距离
    stall_delta: float = 0.0015         # 阈值下限 rad

    # ── 力矩 / 指令上限 ────────────────────────────────────────────────
    max_lead_mm: float = 4.0            # 行进段领先上限 mm（≈ kp × 上限）

    # ── 带设定力的接近段（grasp 的闭合段）的力矩预算 ───────────────────
    # grasp 的闭合段是**带设定力**的，可它这一段仍然是位置帧：撞上工件时力矩由
    # 驱动器自己算 —— ``kp × 领先 + kd × 指令速度``，和设定的力没关系。默认那
    # 条 4 mm 行进段上限在配置默认值下（``kp=100``、``rad_to_mm=75.44``）是
    # 5.30 Nm ≈ 53 N，所以 5 N 的夹取也是拿 53 N 撞上去的。这一段改成把设定力
    # 的力矩预算分给一帧里的三项，撞上工件时压出的力矩就不超过预算 —— 分配见
    # :func:`force_approach_terms`，与 console（litegrip-studio）的接近段同一
    # 条律、同一个数。
    #
    # 边界：预算直接由 ``force_n × UnitConversion.N_TO_NM`` 折算，再乘这个余量。
    # 夹爪的额定力不在这一层拦 —— 那是调用方给的设定值自己的事，而保力段本来也
    # 照设定的力往下发。
    press_safety: float = 0.9           # 预算余量：撞上时实测力落在设定值之下

    # ── 行进段堵转保护（≈7 N） ────────────────────────────────────────
    # 普通移动（``press=True`` 的 open/close）行进段的领先上限是 max_lead_mm，
    # 一旦夹爪在半路被硬挡，``kp × 领先上限`` ≈ 7.6 N·m 会一直压着；而位置
    # 窗口判据对「缓慢变形」的硬停会漏判（结构让位让窗口净位移一直够）。这
    # 一路**独立按力矩保护**：行进段里实测速度明显跟不上指令、且 ``|tau|``
    # 连续 stop_torque_cycles 次过阈 → 判堵转并失力。压紧段（已经贴着标定
    # 限位，``lead_cap`` 已切到 stop_lead_mm）不走这一路 —— 那里本来就该顶着
    # 力矩，硬加力矩门槛会让每一次 open/close 都误触发。
    stop_torque_nm: float = 0.7         # 触发保护的力矩阈值 Nm（≈7 N）
    stop_torque_cycles: int = 3         # 连续这么多个采样点都过阈才触发
    stop_speed_ratio: float = 0.5       # 实测速度低于指令速度这个比例才算「没跟上」
    stop_release_s: float = 0.2         # 触发后失力（kp=kd=tau=0）持续时长 s

    # ── 保力 ───────────────────────────────────────────────────────────
    # 保力帧是**纯力矩源**：kp=kd=0，只有前馈力矩。力控要的是力，位置/速度
    # 增益会让力跟着夹爪的位移和速度走 —— 工件一让位（或闭合侧约 0.010 rad
    # 的粘滑死区一动），``kp × 位移`` 就从设定力里扣掉一截，现象是「先夹到
    # 设定力，过一会儿掉下来」。推导与代价见 GripperActions._hold_force。
    force_n: float = 20.0               # 默认夹持力 N（= 2.0 Nm）
    # 保力力矩的**爬升速率** N/s：保力从进入时飞行中的力矩按这个速率爬到设定值，
    # 而不是一步跳过去、也不是按指数逼近（指数只是把阶跃抹开，第一拍最陡）。一步
    # 踏进接触里就是隔着机构的一次冲击，指爪会被刚碰到的东西弹开；按速率爬升才是
    # 「力均匀地涨、到了设定值就停」。20 N/s 下，从压紧的约 10 N 交接到 20 N 设定值
    # 要半秒，设定值最大到额定 40 N 要两秒。和 console（litegrip-studio）同一个数、
    # 同一个单位，两边保持同步。
    force_ramp_n_s: float = 20.0        # 保力爬升速率 N/s
    hold_interval: float = 0.2          # 保力分片时长 s
    # 保力段的**失控检测**阈值 rad：实测位置越过闭合侧标定限位这么多就失力中止
    # （判据见 closing_travel_excursion_rad，触发后的动作见 _hold_force）。纯力矩
    # 帧里没有位置误差可夹，撤掉力矩是唯一能用的手段，所以这里是判据不是夹子。
    #
    # 这是检测**失控**，不是给保力定精度：正常保力根本走不到这里 —— 夹着工件时
    # 夹爪停在工件上；空载时它顶到真限位就停，而真限位在标定限位附近（zero() 探
    # 测时是压进限位的，所以标定出来的限位比夹爪真正能到的位置还宽约 1 mm，空载
    # 保力的越位量因此接近 0）。0.05 rad 是给「标定略微偏了」和「真限位不在标定
    # 位置上」留的余量，按出厂标定的 rad_to_mm=61.01 折算约 3.05 mm。真出问题时
    # 越位量是毫米级往上走的 —— 2026-10-10 那台是 104.9 mm。
    hold_runaway_rad: float = 0.05
    # [Deprecated] 保力不再用增益（见上）。留着只为兼容老配置，设了也不生效。
    hold_kp: float = 150.0              # 已废弃：保力刚度
    hold_kd: float = 2.0                # 已废弃：保力阻尼

    # ── 使能 ───────────────────────────────────────────────────────────
    enable_retries: int = 3             # 使能重试次数
    enable_retry_interval: float = 0.2  # 使能重试间隔 s

    # ── zero() 标定探测 ────────────────────────────────────────────────
    # 探测顶在限位上的力矩就是 calib_kp × 指令领先量，而领先量本身被
    # _find_limit 限成 calib_step_rad（目标只比实测位置多一步），所以这两个
    # 值同时决定「走多快」和「顶多重」：20 × 0.05 ⇒ 空载推进约 1 Nm，
    # 远低于 DM4310 的峰值。calib_tau_limit 是独立于堵转判据的硬上限。
    calib_kp: float = 20.0              # 低刚度更温和（力矩 = kp × 领先量）
    calib_kd: float = 2.0
    calib_step_rad: float = 0.05        # 每步步进 rad（= 指令领先上限）
    calib_tau_limit: float = 2.0        # 力矩上限 Nm，超过即停
    calib_stall_delta: float = 0.0015   # 标定堵转判据 rad
    calib_stall_cycles: int = 5         # 标定连续堵转次数
    calib_max_iter: int = 200           # 单向步数上限

    # ── 测试/仿真缝 ────────────────────────────────────────────────────
    sleep_fn: Callable[[float], None] = field(
        default=time.sleep, repr=False, compare=False)
    monotonic_fn: Callable[[], float] = field(
        default=time.monotonic, repr=False, compare=False)


# ═══════════════════════════════════════════════════════════════════════════
# 结果类型
# ═══════════════════════════════════════════════════════════════════════════
#
# 都实现 __bool__，所以 ``if gripper.open():`` 这种老写法继续可用。

@dataclass
class MoveProgress:
    """一次进展快照，交给 ``progress`` 回调。"""

    phase: str                          # "move" 或 "hold"
    i: int                              # 当前帧号 / 保力片号
    total_steps: int                    # 总帧数（保力段为 0）
    cmd_rad: float                      # 本帧下发的指令位置
    pos_rad: float                      # 本帧实测位置
    delta_rad: float                    # 距上一次采样的位置增量
    win_delta_rad: Optional[float]      # 窗口净位移（采样点数不够时为 None）
    torque_nm: float                    # 实测力矩
    temperature_coil: int = 0           # 线圈温度 °C


@dataclass
class MoveResult:
    """一次移动的结果。

    ``ok`` 是「这次动作算不算成功」，``__bool__`` 用它。注意含义随目标而变：

    - ``open()`` / ``close()``：成功 = **堵转停在标定限位近旁**（下发的指令
      被封顶在标定限位上，见 :func:`clamp_to_travel`），所以 ``ok=True`` 时
      ``stalled=True`` 而 ``reached`` 基本为 ``False``。半路被工件挡住也算
      堵转，但离标定限位很远，``ok=False``。
    - ``grasp()`` 的闭合段：成功 = 走到空载目标且没堵转，即
      ``reached and not stalled``，与旧行为一致。
    - 定位（``goto`` 一族、``move_at_speed`` 一族，都走
      :meth:`GripperActions.ramp_to`）：成功 = 同上 ``reached and not
      stalled``。半路被挡住、整段一帧都没发出去、或者根本没走到，都是
      ``False`` —— 目标在行程内，「顶在限位上」不是它的成功终点。
    """

    ok: bool                            # 本次动作是否成功（__bool__ 用它）
    reached: bool                       # 末端是否在 reach_tol 内到目标
    stalled: bool                       # 是否判到堵转
    state: GripperState                 # 末端状态
    target_rad: float                   # 目标位置
    limit_rad: float                    # 这一端标定出的机械限位
    final_cmd_rad: float                # 最后一帧下发的指令位置
    steps: int                          # 实际走了多少帧
    # 行进段堵转保护（≈7 N 力矩）触发 —— 触发即失力，``ok`` 必为 False。
    # 与 ``stalled`` 的区别：``stalled`` 也会由「顶到标定限位」成立（那是
    # open/close 的正常成功终点），``protected`` 只报这一路力矩保护。
    protected: bool = False
    # 没到位之后的撤力（只有 ramp_to 会置上）：定位半路被挡住时，最后一帧的
    # 指令停在 ``实测位置 + 领先上限`` 上，留着它驱动器会按约 6.6 N·m 一直压着。
    # 只报「撤了力」，不改变 ``ok``。
    released: bool = False

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class GraspResult:
    """grasp 的结果。"""

    ok: bool                            # 保力是否正常结束（没被故障/中止打断）
    reached: bool                       # 闭合段是否到空载目标位置
    stalled: bool                       # 闭合段是否判到堵转（= 夹到工件）
    state: GripperState                 # 末端状态
    target_rad: float                   # 闭合段的空载目标位置
    force_n: float                      # 实际用的夹持力 N
    cycles: int                         # 保力片数
    # 保力段被**位置护栏**中止：实测位置越过闭合侧标定限位超过
    # MotionConfig.hold_runaway_rad，于是撤力返回。触发即 ``ok=False``。
    # 含义与 :attr:`MoveResult.protected` 对称 —— 那一路守的是移动段的力矩，
    # 这一路守的是保力段的越位。正常夹取不会置位：夹着工件时夹爪停在工件上，
    # 空载时顶到真限位就停。置位说明闭合侧没有东西拦着它。
    travel_guard: bool = False

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class HoldResult:
    """保力段（:meth:`GripperActions._hold_force`）的结果。

    引擎内部的返回类型，``grasp`` 把它折进 :class:`GraspResult`。
    """

    ok: bool                            # 是否保到时间自然结束（没被故障/护栏打断）
    cycles: int                         # 保力片数
    state: Optional[GripperState]       # 最后一次状态
    guarded: bool = False               # 是否被位置护栏中止（详见 GraspResult）

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class EnableResult:
    """enable 的结果。"""

    ok: bool                            # 状态帧是否回读到 err == 1
    state: Optional[GripperState]       # 最后一次状态
    tries: int                          # 实际尝试次数

    def __bool__(self) -> bool:
        return self.ok


@dataclass
class WriteZeroResult:
    """write_zero 的结果。"""

    before_rad: float                   # 写入前的角度
    after_rad: float                    # 重新使能后回读的角度
    tolerance_rad: float = 1e-3         # 回读视为 ~0 的容差

    @property
    def ok(self) -> bool:
        """回读是否 ~0（0xFE 是否被电机接受）。"""
        return abs(self.after_rad) <= self.tolerance_rad

    def __bool__(self) -> bool:
        return self.ok


# ═══════════════════════════════════════════════════════════════════════════
# 目标位置
# ═══════════════════════════════════════════════════════════════════════════

def _check_calibrated(config: GripperConfig) -> None:
    """确认配置真的带上了标定，并有一段非零行程。

    两种限位顺序都合法（``pos_closed_rad`` 可以小于 ``pos_open_rad``，那就是
    反装），所以这里**不**看谁大谁小 —— 只看有没有标定过的实数。没标定的
    配置里所有方向都是猜的，必须拦住。
    """
    if not config.calibrated:
        raise CommandError(
            "配置尚未标定：pos_closed_rad / pos_open_rad 还是占位默认值，"
            "无法判断方向。先 load_calibration()（可选 "
            "CALIB_TEMPLATES[\"reverse\"]）或跑一次 zero()。")
    if abs(config.pos_closed_rad - config.pos_open_rad) <= 1e-6:
        raise CommandError(
            f"行程为零：pos_closed_rad={config.pos_closed_rad} 与 "
            f"pos_open_rad={config.pos_open_rad} 相同，重新标定。")


def clamp_to_travel(config: GripperConfig, q: float) -> float:
    """把位置指令夹进标定行程 ``[pos_closed_rad, pos_open_rad]``。

    两端谁大谁小取决于安装方向（``close_sign``），所以这里取 ``min`` / ``max``，
    不假设顺序 —— 与 :func:`limit_target` 一样，正装、反装通用。

    这是引擎**最后一道**位置护栏：:func:`press_target` 故意把 ``open`` / ``close``
    的**目标**放在标定限位之外，但**下发**的指令不许越过去。指令一旦越过去，夹爪
    就朝那边一直走 —— 真限位不在那儿（标定偏了、机械限位已经损坏、或者本来就
    没有）时，没有东西拦得住它。

    非有限值不归这里兜底：``min`` / ``max`` 与 NaN 的比较全为假，NaN 会被静默
    折到量程的一端。调用方必须在进来之前拒绝非有限值 —— CAN 层
    :meth:`LiteGripCAN._check_motion_command` 已经在做这件事。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` / ``pos_open_rad``）。
        q: 待下发的指令位置 rad。

    Returns:
        夹进标定行程后的位置 rad。
    """
    lo = min(config.pos_closed_rad, config.pos_open_rad)
    hi = max(config.pos_closed_rad, config.pos_open_rad)
    return max(lo, min(hi, q))


def closing_travel_excursion_rad(config: GripperConfig, q: float) -> float:
    """``q`` 越过闭合侧标定限位、朝闭合方向又多走出来的量 rad（没越过为 0）。

    力控段（``grasp`` 的保力段）的护栏用的就是它。那一段下发的是
    ``kp = kd = 0`` 的纯前馈力矩帧：帧里只有力矩，没有位置误差，
    :func:`clamp_to_travel` 那条位置护栏也就无从下手（它夹的是指令位置）。纯力矩源
    能用的手段只剩一个 —— **撤掉力矩**。所以那一段的护栏是**判据**，不是夹子：
    越位即失力，而不是把夹爪按在某个位置上。两条护栏守的是两件事：位置指令那条
    封顶的是**指令**，这一条止住的是**失控**。

    只判闭合侧。帧里下的是 ``close_sign × force``，力矩朝闭合方向压，所以越过闭合
    限位才是失控 —— 那里一旦没有真限位（标定偏了、机械限位已经损坏），就是一路走
    到底。越过张开限位会被同一份力矩自己推回来，不算失控；把那边也算进来只会让
    「松手把夹爪推过张开限位、接着合上」这种正常操作误触发。

    反装（``pos_closed_rad < pos_open_rad``）由 :attr:`GripperConfig.close_sign`
    统一处理，正装反装共用这一份写法。

    配置没标定时**不要用**：占位默认值（``pos_closed_rad=1.14`` /
    ``pos_open_rad=0.0``）描述的不是任何真实行程，由它算出来的越位量没有意义。
    调用方负责先看 :attr:`GripperConfig.calibrated`。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` 与 ``close_sign``）。
        q: 实测位置 rad。

    Returns:
        越位量 rad；没越过为 ``0.0``。
    """
    return max(0.0, config.close_sign * (q - config.pos_closed_rad))


def limit_target(
    config: GripperConfig,
    toward: str,
    margin: float,
) -> Tuple[float, float, float, float]:
    """算出一端的目标位置：标定限位往行程内侧退 ``margin`` 比例的余量。

    不让夹爪真顶到机械限位（那里 kp=100 会压出 ~4 Nm），而是停在限位内侧。

    方向来自 :attr:`GripperConfig.close_sign`，所以反装的机器（
    ``pos_closed_rad < pos_open_rad``）同样成立。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` / ``pos_open_rad``）。
        toward: ``"close"`` 或 ``"open"``。
        margin: 行程余量比例，0.05 = 两端各留 5%。

    Returns:
        ``(目标位置, 标定限位, 余量 rad, 行程 rad)``

    Raises:
        CommandError: 配置还没标定（``calibrated=False``），或行程为零。
            这多半是没加载标定，用了 :class:`GripperConfig` 的默认值。
    """
    _check_calibrated(config)

    s = config.close_sign
    travel = abs(config.pos_closed_rad - config.pos_open_rad)
    margin_rad = margin * travel
    if toward == "close":
        limit = config.pos_closed_rad
        return limit - s * margin_rad, limit, margin_rad, travel
    limit = config.pos_open_rad
    return limit + s * margin_rad, limit, margin_rad, travel


def press_target(
    config: GripperConfig,
    toward: str,
    overshoot: float,
) -> Tuple[float, float, float, float]:
    """算出一端的目标位置：**越过**标定限位 ``overshoot`` 比例的行程。

    与 :func:`limit_target` 相反 —— 目标是「压过去」，让运动靠堵转结束在限位上。
    它算的只是**目标**：真正下发的指令由 :func:`clamp_to_travel` 封顶在标定
    限位，所以终点是标定限位，不是物理限位。标定偏了（或真实限位不在标定位置
    上）只会让夹爪停得更早，不会让它走得更远。

    压紧段的领先上限由 :attr:`MotionConfig.stop_lead_mm` 收窄，接近段的力矩
    因此不超过 ``kp × stop_lead_mm / rad_to_mm``；指令封顶在限位上之后，顶住
    限位时的残余领先更小，力矩只会更低。

    方向来自 :attr:`GripperConfig.close_sign`，正向与反装都成立。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` / ``pos_open_rad``）。
        toward: ``"close"`` 或 ``"open"``。
        overshoot: 越位比例，0.05 = 往限位外侧再走 5% 行程。

    Returns:
        ``(越过限位的目标, 标定限位, 越位 rad, 行程 rad)``

    Raises:
        CommandError: 配置还没标定（``calibrated=False``），或行程为零。
    """
    _check_calibrated(config)

    s = config.close_sign
    travel = abs(config.pos_closed_rad - config.pos_open_rad)
    over_rad = overshoot * travel
    if toward == "close":
        limit = config.pos_closed_rad
        return limit + s * over_rad, limit, over_rad, travel
    limit = config.pos_open_rad
    return limit - s * over_rad, limit, over_rad, travel


def work_limit_target(
    config: GripperConfig,
    work_stroke_mm: float,
) -> Tuple[float, float, float]:
    """张开侧「工作行程」目标：从闭合零点起算 ``work_stroke_mm`` 处的指令位置。

    与 :func:`press_target` 相反 —— 它**不**越位压到机械限位，而是在限位内侧
    留出一段余量（现场口径：两爪行程 85 mm，工作只用到 80 mm，开口端留 5 mm）。
    余量是**显式**的毫米数，不是 :func:`limit_target` 那种按行程比例的 ``margin``。

    方向来自 :attr:`GripperConfig.close_sign`，所以反装的机器同样成立。目标按
    机械行程 clamp，``work_stroke_mm`` 不小于机械行程时就是「张开到底」。

    Args:
        config: 夹爪配置（用 ``pos_closed_rad`` / ``pos_open_rad`` / ``rad_to_mm``）。
        work_stroke_mm: 工作行程 mm（自闭合零点起算）。

    Returns:
        ``(目标 rad, 张开侧标定限位 rad, 实际工作行程 rad)``

    Raises:
        CommandError: 配置还没标定（``calibrated=False``），或行程为零。
    """
    _check_calibrated(config)

    s = config.close_sign
    travel_mm = abs(config.pos_open_rad - config.pos_closed_rad) * config.rad_to_mm
    stroke_mm = max(0.0, min(work_stroke_mm, travel_mm))
    work_rad = stroke_mm / config.rad_to_mm
    return config.pos_closed_rad - s * work_rad, config.pos_open_rad, work_rad


def force_approach_terms(
    kp: float,
    kd: float,
    speed_rad_s: float,
    frame_interval: float,
    budget_nm: float,
    lead_ceiling_rad: float,
) -> Tuple[float, float, float]:
    """把**带设定力**的接近段的力矩预算分给一帧里的三项。

    返回 ``(速度 rad/s, 阻尼, 领先上限 rad)``。

    ``grasp`` 的闭合段带着设定力走，可它这一段还是位置帧：撞上工件时力矩由驱
    动器自己算，是 ``kp × 领先 + kd × 指令速度`` —— 只由这一段的速度和引擎自己
    给的领先决定，与设定的力无关。默认那条 ``max_lead_mm`` 在配置默认值下
    （``kp=100``、``rad_to_mm=75.44``）是 5.30 Nm ≈ 53 N，所以 5 N 的夹取也是
    拿 53 N 撞上去的。

    这里把设定力的力矩预算 ``force_n × UnitConversion.N_TO_NM ×
    MotionConfig.press_safety`` 分给这一帧能出力的每一项，接近段撞上工件时压出
    的力矩就不超过预算：

    * **一帧自己那一格**（``kp × v × dt``）只能用速度封：指令再怎么跟，本帧的
      位移也让指令领先实测一格，撞上就是这么多。所以预算先划出这一格，只够这
      一格时把速度降下来 —— 这是唯一一处设定力仍然决定接近**速度**的地方，而
      且是这一帧的算术，不是策略；
    * **阻尼**（``kd × v``）接着分，按预算收窄，但不高于调用方给的上限；
    * **领先**拿剩下的。它的下限是一帧的位移（``min_cap_rad``，引擎本来就靠这
      条保住斜坡自己那一格），而这个下限的力矩恰好就是上面那一格，所以不会把
      预算撑破。

    顺序是有意的：阻尼先分、位置项（领先）后分，预算不够时先砍领先。反过来先给
    领先、剩下的才给阻尼，在低速端（``kd × v`` 只有一两牛）会把阻尼砍到零，那样
    一帧里只剩位置项，工件让位时力就跟着掉。

    ``budget_nm <= 0``（没带设定力）时原样返回，两个上限都不动。

    Args:
        kp: 位置刚度（驱动器那一档，默认 100）。
        kd: 调用方给的阻尼上限。
        speed_rad_s: 这一段原本的指令速度 rad/s。
        frame_interval: 一帧的时长 s。
        budget_nm: 这一段允许压出的力矩 N·m。
        lead_ceiling_rad: 不进预算时用的领先上限 rad（如 ``max_lead_mm``）。

    Returns:
        ``(速度, 阻尼, 领先上限)``，满足
        ``kp × 领先上限 + 阻尼 × 速度 ≤ 预算``。
    """
    if budget_nm <= 0.0 or kp <= 0.0 or speed_rad_s <= 0.0:
        return speed_rad_s, kd, lead_ceiling_rad

    # 一帧的位移本身就是指令领先实测的下限，速度不能快到它自己就吃掉预算。
    speed_rad_s = min(speed_rad_s, budget_nm / (kp * frame_interval))
    tick_nm = kp * speed_rad_s * frame_interval
    tick_rad = speed_rad_s * frame_interval
    used_kd = max(0.0, min(kd, (budget_nm - tick_nm) / speed_rad_s))
    lead_budget_rad = (budget_nm - used_kd * speed_rad_s) / kp
    return (speed_rad_s, used_kd,
            min(lead_ceiling_rad, max(tick_rad, lead_budget_rad)))


# ═══════════════════════════════════════════════════════════════════════════
# 动作层
# ═══════════════════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════════════════
# 动作层
# ═══════════════════════════════════════════════════════════════════════════

def _toward(value: float, target: float, step: float) -> float:
    """把 ``value`` 朝 ``target`` 挪最多 ``step``，并且**正好落在** ``target`` 上。

    落在靶点上、而不是逼近它，是关键：只按剩余距离的某个比例前进的斜坡永远到不了
    终点，而保力的结果，操作者就是按它最终停在的数值来读的。
    """
    if value < target:
        return min(value + step, target)
    return max(value - step, target)


def _raise_send_failure(g: "LiteGrip") -> None:
    """抛出一个带原因的异常，说明为什么 ``send_mit_frame`` 回了 ``False``。

    原因在 :attr:`LiteGrip.last_send_error` 里，三种情况分开报：

    - 没连上 / 没使能 —— 下层记的就是 ``NotInitializedError``，原样抛回去：
      它的消息说的是该补哪一步，重新包一层只会把那个结论弄丢。
    - 总线写失败 —— 下层记的是它自己那条异常（``OSError`` 等）。这一层看不到
      总线，只能照搬，按 README 的错误表归到 ``CommError``，并把原异常挂在
      ``__cause__`` 上。
    - 什么都没记 —— 替身只回 ``False``，或者失败在别处。照实说「原因未记录」，
      不替下层猜一个「未连接或未使能」：猜错了，诊断就去错了地方。
    """
    error = getattr(g, "last_send_error", None)
    if isinstance(error, NotInitializedError):
        raise error
    if error is None:
        raise CommError("MIT 帧下发失败（原因未记录）")
    raise CommError(f"MIT 帧下发失败：{error!r}") from error


class GripperActions:
    """夹爪的七个动作：``open`` / ``close`` / ``grasp`` / ``ramp_to`` /
    ``zero`` / ``enable`` / ``disable``。

    一般不直接构造，用 :attr:`LiteGrip.actions`::

        with LiteGrip("can0") as g:
            g.load_calibration()
            g.actions.enable()
            g.actions.open()
            g.actions.grasp(force_n=20.0, hold_s=3.0)

    进度回调 ``progress`` 收到 :class:`MoveProgress`，本模块自己不打印任何东西。
    """

    def __init__(self, gripper: "LiteGrip", config: Optional[MotionConfig] = None):
        self._g = gripper
        self.config = config if config is not None else MotionConfig()

    # ═══════════════════════════════════════════════════════════════════
    # 六个接口
    # ═══════════════════════════════════════════════════════════════════

    def open(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """全开。

        默认走到张开侧标定限位上堵转停下，``ok=True`` 时 ``stalled=True``、
        ``reached`` 基本为 ``False``。下发的指令被封顶在标定限位
        （:func:`clamp_to_travel`），所以标定偏了、真实限位不在标定位置上、甚至
        限位已经损坏，夹爪也不会被驱动到标定行程之外。

        但若配置带了**工作行程**（:attr:`GripperConfig.work_stroke_mm` 且小于
        机械行程），只走到那里就停 —— 开口端留出余量，不再走向机械限位。这时是
        一次普通定位（``ok = reached and not stalled``），``limit_rad`` 仍是
        张开侧标定限位，示意目标停在它内侧。
        """
        cfg = self.config
        gcfg = self._g.config
        speed = cfg.speed_mm_s if speed_mm_s is None else speed_mm_s
        travel_mm = abs(gcfg.pos_open_rad - gcfg.pos_closed_rad) * gcfg.rad_to_mm
        if 0.0 < gcfg.work_stroke_mm < travel_mm:
            target, _limit, _rad = work_limit_target(gcfg, gcfg.work_stroke_mm)
            return self._move_to_limit("open", speed, target_rad=target,
                                       progress=progress)
        return self._move_to_limit("open", speed, press=True, progress=progress)

    def close(
        self,
        speed_mm_s: Optional[float] = None,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """全合：走到闭合侧标定限位上堵转停下。

        下发的指令被封顶在标定限位（:func:`clamp_to_travel`），所以标定偏了、
        真实限位不在标定位置上、甚至限位已经损坏，夹爪也不会被驱动到标定行程
        之外。``ok=True`` 时 ``stalled=True``、``reached`` 基本为 ``False`` ——
        见 :class:`MoveResult`。
        """
        speed = self.config.speed_mm_s if speed_mm_s is None else speed_mm_s
        return self._move_to_limit("close", speed, press=True, progress=progress)

    def grasp(
        self,
        force_n: Optional[float] = None,
        hold_s: float = 0.0,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> GraspResult:
        """夹取：先闭合到工件上（或空载到限位），再持续输出 ``force_n`` 大小的力。

        闭合段**带着设定力走**：它的力矩预算就是这一次夹取的力（见
        :func:`force_approach_terms`），撞上工件时压出的力矩不超过它 —— 空载时
        它才一路走到底。

        **交接只在「到位或顶上东西」时发生**（``reached or stalled``）。这两条
        在真机上是同一件事的两面：夹爪都停着。这不是保守，是唯一的正确条件 ——
        保力段是纯力矩源，帧里没有位置项也没有速度项，把它接到一个**还在自由
        行程里**的夹爪上，位置环一撤它就只有加速这一条路（2026-10-13 实测：
        0.82 rad/s → 5.12 rad/s，312 mm/s，且到 ``hold_s`` 用完仍在加速）。
        所以闭合段的终止条件也是位置而不是帧数：``ramp_steps`` 跑完只说明排程
        跑完了，不等于夹爪到了 —— 带设定力时领先被压到零点几毫米，它追不追得上
        排程由摩擦和读数延迟定。两条都没成立时**如实报 ``ok=False``** 并撤力
        返回（``reached`` / ``stalled`` 都是 ``False``，``cycles=0``），不放跑。

        保力段是纯力矩源，没有位置指令可夹，所以它有自己的一条护栏
        （:attr:`MotionConfig.hold_runaway_rad`）：越出闭合侧标定限位太多就撤力
        返回，结果里 ``ok=False`` / ``travel_guard=True``。正常夹取不会触发。

        Args:
            force_n: 夹持力 N，``None`` = 用 :attr:`MotionConfig.force_n`。
                按 SDK 近似换算 1 N = 0.1 Nm。
            hold_s: 保力时长 s。``0`` = 一直保到出错、被护栏中止或 Ctrl+C。
            progress: 进度回调。

        Returns:
            :class:`GraspResult`。夹住工件时 ``stalled=True`` 且
            ``reached=False``（压不到空载目标位置是正常的）。闭合段结束时夹爪
            既没到位也没堵转时 ``ok=False``、``cycles=0`` —— 见上。

        Raises:
            CommandError: ``force_n`` / ``hold_s`` 不是有限数。两条都不是
                「夹得轻/夹得久」的边界，而是没有意义的输入：NaN 的力会让闭合段
                的力矩预算（:func:`force_approach_terms`）整段变成 NaN，NaN 的
                ``hold_s`` 会让保力段一帧不保就返回 —— 偏偏报「成功」。
                一直保力请用 ``hold_s=0``，不是 ``inf``。
        """
        cfg = self.config
        force = cfg.force_n if force_n is None else force_n
        for name, value in (("force_n", force), ("hold_s", hold_s)):
            if not math.isfinite(value):
                raise CommandError(
                    f"grasp 的 {name} 不是有限数：{value!r}。拒绝下发。")

        # 顺序与 ramp_to 一致：先「使能了吗」再「标定了吗」，两条都是拒绝而不
        # 是夹紧。``target_rad`` 那条支路不走 limit_target，标定检查得自己来；
        # 少了它，未标定时的拒绝会静默消失。
        self._g._check_enabled()
        gcfg = self._g.config
        _check_calibrated(gcfg)

        # 闭合段瞄的是**闭合侧标定限位本身**，不是 limit_target 在限位内侧留的
        # 那 5% 行程。留白那几毫米是自由行程：夹爪到了点上、前面什么都没有，保
        # 力段随后只能用纯力矩把它推过去 —— 那是真机上「末端飞快一下」的一半来
        # 源。瞄限位时夹爪是顶在东西上或停在限位上的，交接从静止开始。
        # ``target_rad`` 会强制 press=False，ok 与 reached 的判据都不动。
        move = self._move_to_limit(
            "close", cfg.grasp_speed_mm_s,
            target_rad=gcfg.pos_closed_rad,
            force_n=force, progress=progress)
        if not (move.reached or move.stalled):
            # 既没到位也没顶上东西 —— 夹爪还在自由行程里，而位置环是此刻唯一
            # 拦着它的东西（kp × 领先 + kd × 速度误差）。保力段的帧里这两项全
            # 没有，交过去就是把设定力矩直接接到一个还在走的夹爪上：真机实测速
            # 度从被速度环限住的 0.82 rad/s 跳到 5.12 rad/s，且到 hold_s 用完
            # 还在加速。宁可如实报失败，也不交。
            self._release(move.state.position_rad)
            log.warning("夹取未交接：闭合段结束时夹爪既未到位也未堵转（停在 "
                        "%+.5f rad，目标 %+.5f rad）—— 它还在自由行程里，而保力"
                        "段没有位置项也没有速度项，交过去会一路加速。撤力返回。",
                        move.state.position_rad, move.target_rad)
            return GraspResult(
                ok=False,
                reached=move.reached,
                stalled=move.stalled,
                state=move.state,
                target_rad=move.target_rad,
                force_n=force,
                cycles=0,
                travel_guard=False,
            )
        hold = self._hold_force(force, hold_s, progress=progress)
        return GraspResult(
            ok=hold.ok,
            reached=move.reached,
            stalled=move.stalled,
            state=hold.state if hold.state is not None else move.state,
            target_rad=move.target_rad,
            force_n=force,
            cycles=hold.cycles,
            travel_guard=hold.guarded,
        )

    def ramp_to(
        self,
        target_rad: float,
        speed_mm_s: float = 0.0,
        *,
        duration_s: Optional[float] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """斜坡走到行程内的任意一点：普通定位，不顶限位。

        ``open`` / ``close`` 的终点在标定限位外侧（靠堵转停下）；这里走的是
        ``frame_interval`` 一格的连续斜坡，每一帧的指令领先实测位置不超过
        :attr:`MotionConfig.max_lead_mm`，指令力矩因此有界在 ``kp × 领先上限``
        附近（默认 4 mm、kp=100 时约 6.6 N·m，远低于 DM4310 的 ±10 N·m 帧量程）。

        这个上限不是装饰。kp=100 下误差过约 0.1 rad 就把帧量程顶到饱和，而饱和
        的那一帧会冲过目标 —— 2026-10-12 真机上 ``goto(40.0, duration=0.5)``
        冲过目标 42.7 mm、越过标定闭合零点 2.75 mm，就是这么来的。

        到位判据是 :attr:`MotionConfig.reach_tol`：``ok = reached and not
        stalled``。半路被挡住、整段一帧都没发出去、或者根本没走到，都是
        ``ok=False``。

        与 ``open`` / ``close`` 还有一处不同：**没到位就撤力**。压紧段那点领先是
        意图，普通定位半路停住的领先不是 —— 最后一帧的指令停在「实测位置 + 领先
        上限」上，留在总线上驱动器会按约 6.6 N·m 一直压着。所以 ``ok=False`` 时
        补一段 ``kp=kd=tau=0`` 的释放帧，并把 ``released`` 置上。

        Args:
            target_rad: 目标位置 rad（调用方负责先 clamp 进标定行程）。
            speed_mm_s: 斜坡速度 mm/s，必须为正。与 ``duration_s`` 二选一。
            duration_s: 整段斜坡的时长 s，速度由实测距离反推。
            kp: 这一段的刚度，``None`` = :attr:`GripperConfig.kp`。
            kd: 这一段的阻尼，``None`` = :attr:`GripperConfig.kd`。
            progress: 进度回调。

        Raises:
            CommandError: 目标不是有限数、没标定、速度不合法，或者速度与时长
                同时给了。
            NotInitializedError: 未连接或未使能。
        """
        g = self._g
        g._check_enabled()
        gcfg = g.config
        _check_calibrated(gcfg)

        if not math.isfinite(target_rad):
            raise CommandError(
                f"目标位置不是有限数：target_rad={target_rad!r}。拒绝下发："
                "NaN 一进斜坡算术就会变成帧数。")
        if duration_s is None and not (math.isfinite(speed_mm_s)
                                       and speed_mm_s > 0.0):
            raise CommandError(
                f"斜坡速度必须是正的有限数：speed_mm_s={speed_mm_s!r}；或者给 "
                "duration_s 说明这段路走多久。速度为 0 时斜坡只剩一帧，而那一帧"
                "就是阶跃 —— 正是这条路要消掉的东西。")

        # ``toward`` 只用来给这次移动取名（上报的 ``limit_rad`` 与日志）：目标是
        # 行程内一点，按它离哪一端的标定限位更近取名。反装时两端的数值顺序是反
        # 的，所以比距离，不比大小。
        toward = ("close"
                  if abs(target_rad - gcfg.pos_closed_rad)
                  <= abs(target_rad - gcfg.pos_open_rad) else "open")

        res = self._move_to_limit(toward, speed_mm_s, target_rad=target_rad,
                                  duration_s=duration_s, kp=kp, kd=kd,
                                  progress=progress)
        if not res.ok and not res.protected:
            # 没到位就没到位：不能让「压着误差」的那一帧留在总线上。
            self._release(res.state.position_rad)
            res.released = True
        return res

    def zero(self) -> CalibrationData:
        """完整标定：探闭合 + 张开两个限位，算出行程与换算系数，并存盘。

        过程中夹爪会主动顶住两端机械限位（低刚度探测）。确保行程内无物。
        探测自带两道护栏：指令领先量不超过 ``calib_step_rad``，且 ``|tau|`` 一到
        ``calib_tau_limit`` 立即停止推进 —— 顶住限位时结构让位（背隙/弹性变形）
        会让位置读数一直在动，只靠「位置不再变化」是停不下来的。

        结果落在 :func:`~litegrip.default_calib_path` 那个文件里，但无参
        ``load_calibration()`` 只读出厂标定 —— 下次要用这份，得写
        ``load_calibration(path=default_calib_path(channel))`` 或设
        ``LITEGRIP_CALIB``。

        Returns:
            :class:`CalibrationData`。
        """
        cfg = self.config
        data = self._g.calibrate(
            kp=cfg.calib_kp,
            kd=cfg.calib_kd,
            step_rad=cfg.calib_step_rad,
            stall_delta=cfg.calib_stall_delta,
            stall_cycles=cfg.calib_stall_cycles,
            max_iter=cfg.calib_max_iter,
            tau_limit=cfg.calib_tau_limit,
        )
        self._g.save_calibration()
        return data

    def enable(self, retries: Optional[int] = None) -> EnableResult:
        """反复 enable 直到状态帧回读到 ``err == 1``（真使能）。

        为什么需要重试：``enable`` 是单向命令、无确认，CAN 上丢一帧就白发了。
        这里把「发 enable → 回读状态帧 → 不是 1 就重发」做成显式重试。
        ``err`` 属于真实故障（非 0/1）时先 ``clear_fault()`` 再重试。

        Args:
            retries: 重试次数，``None`` = 用 :attr:`MotionConfig.enable_retries`。

        Returns:
            :class:`EnableResult`（``ok`` / ``state`` / ``tries``）。
        """
        cfg = self.config
        tries_max = cfg.enable_retries if retries is None else retries
        st: Optional[GripperState] = None

        for i in range(1, tries_max + 1):
            try:
                self._g._enable_once()
            except LiteGripError as e:
                log.warning("enable() 第 %d/%d 次抛错：%s: %s",
                            i, tries_max, type(e).__name__, e)

            st = self._g.get_state()
            if st.error_code == 1:
                return EnableResult(ok=True, state=st, tries=i)

            if st.error_code not in (0, 1):
                # 真实故障（欠压/过流/过温等）：先清故障再重试
                try:
                    self._g.clear_fault()
                except LiteGripError as e:
                    log.warning("clear_fault() 失败：%s", e)

            if i < tries_max:
                log.warning("第 %d/%d 次未使能（状态帧 err=%d，0=未使能）"
                            "—— %.2fs 后重发 enable",
                            i, tries_max, st.error_code,
                            cfg.enable_retry_interval)
                cfg.sleep_fn(cfg.enable_retry_interval)

        return EnableResult(ok=False, state=st, tries=tries_max)

    def disable(self) -> bool:
        """失能电机。"""
        return self._g._disable_once()

    # ═══════════════════════════════════════════════════════════════════
    # 引擎
    # ═══════════════════════════════════════════════════════════════════

    def _emit(self, q: float, dq: float = 0.0, tau: float = 0.0,
              kp: Optional[float] = None, kd: Optional[float] = None) -> float:
        """下发一帧 MIT 并等待一帧的时间。

        位置先过 :func:`clamp_to_travel`：这是引擎所有帧的必经之路，所以「指令
        不越标定行程」是引擎级不变式，而不是某一处算术的性质。``open`` /
        ``close`` 的目标本来就在限位外（:func:`press_target`），夹的就是它。

        Returns:
            真正下发的那个 ``q``（夹过之后的值）—— 调用方拿它填
            ``MoveResult.final_cmd_rad`` / ``MoveProgress.cmd_rad``，报的是发出去
            的指令，不是算出来又没发的。

        Raises:
            NotInitializedError: 夹爪没连上，或者没使能 —— 原样抛出
                :attr:`LiteGrip.last_send_error` 里记的那一条，它说的是该补哪
                一步（``connect()`` / ``enable()``）。
            CommError: 帧没发出去，而下层给的原因是传输层失败（如
                ``OSError``），或者什么都没给。消息里带原样的原因 —— 这一层
                看不到总线，替下层猜一个「未连接或未使能」就是谎报。
        """
        cfg = self.config
        g = self._g
        q = clamp_to_travel(g.config, q)
        sent = g.send_mit_frame(
            q,
            g.config.kp if kp is None else kp,
            g.config.kd if kd is None else kd,
            dq=dq,
            tau=tau,
        )
        if not sent:
            _raise_send_failure(g)
        cfg.sleep_fn(cfg.frame_interval)
        return q

    def _release(self, pos: float) -> int:
        """失力：连发 :attr:`MotionConfig.stop_release_s` 的 ``kp=kd=tau=0`` 帧。

        两处保护性中止（行进段的力矩保护、保力段的位置护栏）共用这一份 —— 触发即
        失力，夹爪能被手掰动，而不是继续压着。``q`` 用当前读数：零增益下 ``q`` 不
        产生任何力，只是给个位置合理的指令，免得下游把它当成一次正常定位。

        Args:
            pos: 当前实测位置 rad，作为释放帧的 ``q``。

        Returns:
            实际下发的帧数。
        """
        cfg = self.config
        frames = max(1, int(round(cfg.stop_release_s / cfg.frame_interval)))
        for _ in range(frames):
            self._emit(pos, 0.0, 0.0, kp=0.0, kd=0.0)
        return frames

    def _move_to_limit(
        self,
        toward: str,
        speed_mm_s: float = 0.0,
        *,
        press: bool = False,
        target_rad: Optional[float] = None,
        duration_s: Optional[float] = None,
        force_n: Optional[float] = None,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """走到一端的限位：整段是一条 frame_interval 一格的连续斜坡（MIT 帧 +
        速度前馈），边走边按位置判堵转。

        ``press=False``：目标是「限位内侧留 :attr:`MotionConfig.margin`」，
        到位即成功。**现在没有调用方走这一条** —— ``open`` / ``close`` 走
        ``press=True``，``grasp`` 和 ``ramp_to`` 一族给 ``target_rad``，所以
        ``margin`` 目前只在直接调用本方法时起作用。
        ``press=True``：目标是「越过限位 :attr:`MotionConfig.press_overshoot`」，
        靠堵转停在限位上；距限位 :attr:`MotionConfig.press_zone_mm` 之内领先
        上限切到 :attr:`MotionConfig.stop_lead_mm`，接近段的力矩有界。

        无论走哪一条，每一帧真正下发的指令都经 :func:`clamp_to_travel` 封顶在
        标定行程内（见 :meth:`_emit`）—— 目标可以越位，指令不许。

        ``target_rad``：显式目标（如 :func:`work_limit_target` 算出的工作行程
        点）。给了它就走**普通定位**语义（``press`` 视为 False），``limit`` 取该
        方向的标定限位用于上报。

        ``duration_s``：整段斜坡**走多久**，速度由实测距离除以它反推 —— 距离
        要读了状态才知道，所以它必须在同一个 ``before`` 读数上算，不能在调用方
        先算一遍。``speed_mm_s`` 与它二选一（两个都给就拒绝，不是忽略一个）。

        ``force_n``：这一段**带着多大的力走**。``None`` = 不带（普通移动，压出
        多少由领先上限和速度决定）。给了它，它的力矩预算就分给这一帧的三项（速
        度、阻尼、领先，见 :func:`force_approach_terms`），所以撞上工件时压出的
        力矩不超过这个设定值 —— ``grasp`` 的闭合段给的就是它这次的夹持力。

        带设定力时段的**终止条件也换了一条**：判据是「到位或顶上」，``ramp_steps``
        只是排程、不是它的终止条件。领先被预算压小之后夹爪追不上排程是常态，按帧
        数收尾会在它还差几十毫米时把它交给下一段（真机实测差 21 mm），而那一段
        如果是纯力矩的保力段，接下来就是一路加速。所以这一段的帧数只作兜底，上界
        取「按仍然算进展的最慢速度走完全程要几帧」。

        ``kp`` / ``kd``：这一段的增益，``None`` = 用 :attr:`GripperConfig.kp` /
        ``.kd``。``open`` / ``close`` / ``grasp`` 不给（就是配置值），定位那几条
        透传调用方的。

        返回 :class:`MoveResult`。
        """
        cfg = self.config
        g = self._g
        g._check_enabled()
        gcfg = g.config

        if not math.isfinite(speed_mm_s):
            raise CommandError(
                f"移动速度不是有限数：speed_mm_s={speed_mm_s!r}。拒绝下发："
                "斜坡的帧数与速度前馈都由它算出来。")
        if duration_s is not None:
            if speed_mm_s > 0.0:
                raise CommandError(
                    f"speed_mm_s={speed_mm_s!r} 与 duration_s={duration_s!r} "
                    "只能给一个：前者说「走多快」，后者说「这段路走多久」。")
            if not math.isfinite(duration_s) or duration_s <= 0.0:
                raise CommandError(
                    f"duration_s 必须是正的有限数：{duration_s!r}。速度由实测"
                    "距离除以它算出来 —— 0 或负数没有对应的斜坡。")
            if force_n is not None:
                raise CommandError(
                    "duration_s 与 force_n 不能同时给：带设定力的接近段，速度"
                    "预算由设定力分配（见 force_approach_terms），先定时长会把"
                    "那条预算反过来推翻。")

        if target_rad is not None:
            press = False
            limit = (gcfg.pos_closed_rad if toward == "close"
                     else gcfg.pos_open_rad)
            target = target_rad
        elif press:
            target, limit, _over_rad, _travel = press_target(
                gcfg, toward, cfg.press_overshoot)
        else:
            target, limit, _margin_rad, _travel = limit_target(
                gcfg, toward, cfg.margin)

        before = g.get_state()
        dist_rad = target - before.position_rad
        dist_mm = abs(dist_rad) * gcfg.rad_to_mm
        sign = 1.0 if dist_rad >= 0 else -1.0
        if duration_s is not None:
            # 「这段路走多久」：距离读了状态才知道，所以速度在这里才反推得出。
            # dist_mm == 0 时速度为 0 —— 斜坡退化成一帧 + 保压，正好是「已经在
            # 那儿了」，不必为它写特例。
            speed_mm_s = dist_mm / duration_s
        speed_rad_s = speed_mm_s / gcfg.rad_to_mm

        interval = cfg.frame_interval
        # 不带力的移动用配置里那条行进段上限；带了力，上限和这一帧的阻尼、速
        # 度一起由设定值的预算定（见 force_approach_terms）。
        kp_frame = gcfg.kp if kp is None else kp
        kd_frame = gcfg.kd if kd is None else kd
        lead_ceiling_rad = cfg.max_lead_mm / gcfg.rad_to_mm
        if force_n is not None:
            speed_rad_s, kd_frame, lead_ceiling_rad = force_approach_terms(
                kp=kp_frame,
                kd=kd_frame,
                speed_rad_s=speed_rad_s,
                frame_interval=interval,
                budget_nm=force_n * UnitConversion.N_TO_NM * cfg.press_safety,
                lead_ceiling_rad=lead_ceiling_rad,
            )
            # 预算只降不升，但降下来以后斜坡的帧数、速度前馈和堵转阈值都得跟着
            # 走那一条速度，否则斜坡跑完了夹爪还在半路。
            speed_mm_s = speed_rad_s * gcfg.rad_to_mm

        ramp_s = dist_mm / speed_mm_s if speed_mm_s > 0 else 0.0
        ramp_steps = max(1, int(round(ramp_s / interval)))
        settle_steps = max(1, int(round(cfg.settle_s / interval)))
        total_steps = ramp_steps + settle_steps

        sample_every = max(1, int(round(cfg.sample_interval / interval)))
        win = max(cfg.stall_cycles, 1)
        win_s = win * sample_every * interval
        win_expect_rad = speed_rad_s * win_s
        win_thresh_rad = max(cfg.stall_delta, cfg.stall_ratio * win_expect_rad)
        # 帧数上界。不带力的移动按排程走，排程跑完就是它的终止条件之一；带设定
        # 力的接近段不行 —— 领先被预算压小（见 force_approach_terms），夹爪追不
        # 追得上排程由摩擦和读数延迟定，``ramp_steps`` 到了它可能还差几十毫米。
        # 所以这一段的帧数只是兜底，上界取「按仍然算进展的最慢速度走完全程要几
        # 帧」：比这更慢时堵转判据必然已经在 win_s 内触发，这个上界到不了。
        ceiling = total_steps
        if force_n is not None:
            floor_rad_s = win_thresh_rad / win_s if win_s > 0.0 else 0.0
            if 0.0 < floor_rad_s < speed_rad_s:
                ceiling = max(total_steps,
                              int(math.ceil(abs(dist_rad) / floor_rad_s
                                            / interval)) + settle_steps)
        # 「这一趟本来就没事可做」不判堵转。目标落在到位容差里时，夹爪原地不动
        # 就是到位，读成堵转会把一次成功的定位报成失败 —— 而它会真的发生：排程
        # 的帧数由 duration 定、不随距离缩短，间距趋近 0 时排程速度也趋近 0，
        # 整段斜坡都是「不动」。`home()` 紧跟在 `close()` 之后（两个都瞄着闭合
        # 端，只剩一点点齿隙）就是这个形状。压紧路径不受影响：那里目标在限位
        # 外侧，「到了还不动」正是要判的堵转。
        judge_stall = press or dist_mm > cfg.reach_tol * gcfg.rad_to_mm

        # 两档领先上限：行进段用 max_lead_mm 破静摩擦，贴近限位后收窄到
        # stop_lead_mm，压紧力矩 ≈ kp × stop_lead_mm。下限都是一帧的位移，
        # 免得斜坡自己那一格被切掉。
        min_cap_rad = speed_rad_s * interval
        travel_cap_rad = max(lead_ceiling_rad, min_cap_rad)
        stop_cap_rad = max(cfg.stop_lead_mm / gcfg.rad_to_mm, min_cap_rad)
        press_zone_rad = cfg.press_zone_mm / gcfg.rad_to_mm

        log.info("%s %.4f → %.4f rad（%.1f mm/s，%.1f mm，%d+%d 帧，"
                 "堵转阈值 %.5f rad，领先上限 %.5f/%.5f rad，"
                 "标定行程 [%.4f, %.4f] 封顶%s%s%s）",
                 "闭合" if toward == "close" else "张开",
                 before.position_rad, target, speed_mm_s, dist_mm,
                 ramp_steps, settle_steps, win_thresh_rad,
                 travel_cap_rad, stop_cap_rad,
                 min(gcfg.pos_closed_rad, gcfg.pos_open_rad),
                 max(gcfg.pos_closed_rad, gcfg.pos_open_rad),
                 "，顶限位" if press else "",
                 "" if force_n is None else
                 "，力预算 %.3f Nm（kd %.2f）" % (
                     force_n * UnitConversion.N_TO_NM * cfg.press_safety,
                     kd_frame),
                 "" if ceiling == total_steps else
                 "，帧数上界 %d（兜底，见上）" % ceiling)

        hist = [before.position_rad]
        stalled = False
        protected = False
        over_torque = 0                          # 连续过阈的采样点数
        prev_sample_pos = before.position_rad
        st = before
        last_cmd = before.position_rad
        last_i = 0
        arrived_at = None                        # 首次进入顶住段的帧号（见下）
        clamped_once = False                     # 本次移动只记一条封顶 debug 日志

        for i in range(1, ceiling + 1):
            st = g.get_state(wait=False)
            pos = st.position_rad
            # 排程相位。不带力时按计时切 —— ``ramp_steps`` 就是排程跑完的那一
            # 帧。带设定力时不行：领先被预算压小，夹爪落在排程后面是常态，而
            # ``ramp_steps`` 到了不等于它到了，按计时切会在它还在自由行程时就
            # 判它进了保压段（不再判堵转、撤掉速度前馈）。所以这一段由**位置**
            # 说：没到目标就还在行进段，到了才进保压。ramp_steps 之后 q_sched
            # 停在 target，下面两种写法一致。
            arrived = abs(pos - target) < cfg.reach_tol
            in_ramp = i <= ramp_steps or (force_n is not None and not arrived)
            if in_ramp:
                q_sched = before.position_rad + dist_rad * min(1.0, i / ramp_steps)
                dq = sign * speed_rad_s
            else:
                # 到位后的顶住段（``q_sched = target``、``dq = 0``）：把还在动
                # 的夹爪收住。带设定力时这段同时是**交接前的静止窗口** —— 顶够
                # ``settle_steps`` 帧之后位置环的任务就完了，保力段从静止接手
                # （真机事故就是没等这一段，把 0.82 rad/s 的夹爪直接交给了纯
                # 力矩源）。不带力时下面这个 break 到不了：循环上界就是
                # ramp_steps + settle_steps。
                if arrived_at is None:
                    arrived_at = i
                if i - arrived_at >= settle_steps:
                    break
                q_sched = target                   # 保压：原地顶住目标
                dq = 0.0
            # 压紧段判据按 q_sched 算：斜坡上贴近限位、保压段越过限位，
            # 两种情况都要切到 stop_lead_mm，否则保压段领先会涨到
            # kp × 越位量（≈8 Nm）。
            lead_cap_rad = travel_cap_rad
            if press and (limit - q_sched) * sign <= press_zone_rad:
                lead_cap_rad = stop_cap_rad
            lead = (q_sched - pos) * sign
            cmd = pos + sign * lead_cap_rad if lead > lead_cap_rad else q_sched
            # _emit 会把位置夹进标定行程，返回值才是真正下发的指令：领先项
            # 会让 cmd 跑到限位外侧，尤其 press=True 时（目标本身就在外侧）。
            last_cmd = self._emit(cmd, dq, 0.0, kp=kp_frame, kd=kd_frame)
            if last_cmd != cmd and not clamped_once:
                clamped_once = True
                log.debug("%s 指令封顶标定行程：%.5f → %.5f rad（标定 [%.5f, %.5f]）",
                          toward, cmd, last_cmd,
                          min(gcfg.pos_closed_rad, gcfg.pos_open_rad),
                          max(gcfg.pos_closed_rad, gcfg.pos_open_rad))
            last_i = i

            if i % sample_every and i != ceiling:
                continue
            # 本采样的实测速度（rad/s）：用于堵转力矩保护的「有没有跟上」佐证。
            rate_rad_s = abs(pos - prev_sample_pos) / cfg.sample_interval
            prev_sample_pos = pos
            hist.append(pos)
            win_delta = (abs(hist[-1] - hist[-1 - win])
                         if len(hist) > win else None)
            if progress is not None:
                progress(MoveProgress(
                    phase="move",
                    i=i,
                    total_steps=ceiling,
                    cmd_rad=last_cmd,
                    pos_rad=pos,
                    delta_rad=hist[-1] - hist[-2],
                    win_delta_rad=win_delta,
                    torque_nm=st.torque_nm,
                    temperature_coil=st.temperature_coil,
                ))
            # 保压段 (i > ramp_steps) 本来就该不动，所以默认不判堵转。
            # 但 press=True 时保压段的**目标**在限位外侧（下发的指令被夹在限位上），
            # 夹爪到了那里就该顶着不动 —— 那里的「不动」正是我们要的堵转。而且
            # 越位量（5% 行程）往往短于一个采样窗口，只在斜坡段判会永远判不到，
            # 动作得白跑完保压段。
            if (judge_stall and win_delta is not None
                    and (press or in_ramp)
                    and win_delta < win_thresh_rad):
                stalled = True
                log.info("堵转：最近 %d 次采样(%.2f s)净位移仅 %.5f rad "
                         "(< %.5f)，停在 %+.5f rad",
                         win, win_s, win_delta, win_thresh_rad, pos)
                break

            # 行进段堵转力矩保护（≈7 N）：与位置窗口判据**互补** —— 位置窗口对
            # 「硬停但结构一直缓慢让位」会漏判（净位移始终够），这一路只看力矩。
            # 门槛只在**行进段**生效（leading 还是 max_lead_mm）：那里持续高力矩
            # 意味着真的顶上了东西。压紧段的领先已收窄到 stop_lead_mm，本来就该
            # 顶着力矩，硬加力矩门槛会让每一次 open/close 都误触发。
            if press and lead_cap_rad == travel_cap_rad and speed_rad_s > 0.0:
                slow = rate_rad_s < cfg.stop_speed_ratio * speed_rad_s
                if abs(st.torque_nm) >= cfg.stop_torque_nm and slow:
                    over_torque += 1
                else:
                    over_torque = 0
                if over_torque >= cfg.stop_torque_cycles:
                    protected = True
                    stalled = True
                    log.info("堵转保护：|tau|=%.3f Nm ≥ %.3f 且速度 %.4f rad/s 只"
                             "有指令的 %.0f%%，连续 %d 次采样 —— 判堵转并失力",
                             st.torque_nm, cfg.stop_torque_nm, rate_rad_s,
                             100.0 * rate_rad_s / speed_rad_s, over_torque)
                    break

        if protected:
            self._release(pos)

        st = g.get_state()                         # 阻塞等一帧新状态再判到位
        reached = abs(st.position_rad - target) < cfg.reach_tol
        if press:
            # 顶到限位（堵转 + 停在标定限位附近）才算成功；半路撞工件
            # 也是堵转，但离限位很远 —— 保留这个区分，不丢信号。
            ok = stalled and abs(st.position_rad - limit) <= cfg.stop_tol
        else:
            ok = reached and not stalled
        if protected:
            ok = False                             # 保护性堵转永远不算成功
        return MoveResult(
            ok=ok,
            reached=reached,
            stalled=stalled,
            state=st,
            target_rad=target,
            limit_rad=limit,
            final_cmd_rad=last_cmd,
            steps=last_i,
            protected=protected,
        )

    def _hold_force(
        self,
        force_n: float,
        hold_s: float,
        *,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> HoldResult:
        """持续输出夹持力：整段只下发前馈力矩。

        帧里 ``kp=kd=0`` —— 保力要的是**力**，而 MIT 律里 ``kp × (q - 实测位置)``
        与 ``kd × (0 - 实测速度)`` 都随夹爪的位置和速度变化：工件在设定力下让位
        （或闭合侧约 0.010 rad 的粘滑死区走一格），实测位置就往前挪，``kp × 位移``
        立刻从前馈里扣掉一截，读数表现为「先夹到设定力，过一会儿掉到某个更小的
        值」。上一版锚在 200 ms 前的位置读数上、``kp=150``，工件以 3 mm/s 让位就能
        把 20 N 读成 5 N。所以**不要**为了「顶得更硬」把增益加回来；
        :attr:`MotionConfig.hold_kp` / :attr:`MotionConfig.hold_kd` 已废弃，设了也不
        生效。

        力矩不是一步跳到设定值，而是按 :attr:`MotionConfig.force_ramp_n_s`
        （N/s）从**进入保力时飞行中的力矩**爬上去，且每下发一帧走一步、正好落在
        设定值上。这一步不是可有可无的：交接时电机的力矩本就带着闭合压紧量
        （真机上约 10 N），一步跳到设定值就是隔着机构的一次冲击，指爪会被刚碰到
        的东西弹开 —— 现场看到的是「夹爪停在 10 N，然后跳到 20 N，边跳边往里
        收」。从飞行值开始爬，交接是连续的；爬升率是常量，力就是均匀地涨。定的是
        **速率**不是时长：按时长定的斜坡第一拍最陡，等于带慢尾的阶跃。

        每片 :attr:`MotionConfig.hold_interval` 回读一次状态查故障（一帧一步的爬升
        都在片内完成，见上面的速率）。

        代价：零增益下夹爪可以被外力推动，所以保力段**没有位置指令可夹**
        （:func:`closing_travel_excursion_rad` 的 docstring 讲了为什么）。这一路
        的护栏因此是**判据**：每回读一片就看实测位置是否越出闭合侧标定限位超过
        :attr:`MotionConfig.hold_runaway_rad`，越了就撤力返回。空载时它仍然会顶
        到真限位（和 ``close()`` 的压紧段一样，只是力矩小得多），但真限位不在那
        儿时不会再一路走到底 —— 那正是 2026-10-10 把夹爪顶坏的工况。

        护栏在**两处**判：进循环之前（已经在护栏之外就直接失力，不往里再推）和
        每个回读片。配置没标定时不启用 —— 占位限位描述的不是真实行程。经
        ``grasp()`` 进来时标定一定在：它的闭合段先过 ``_move_to_limit``。

        ``hold_s <= 0`` 表示不限时长（直到出错、被护栏中止或 Ctrl+C）。

        Returns:
            :class:`HoldResult`。
        """
        cfg = self.config
        g = self._g
        gcfg = g.config
        # 夹紧方向的力矩符号随安装方向翻转：正装时 rad 增大是闭合，
        # 反装时反过来。力矩大小不变，只是要让前馈往「夹」而不是「撑」。
        target_nm = gcfg.close_sign * force_n * UnitConversion.N_TO_NM
        # 位置护栏。没标定时不启用：占位限位（pos_closed_rad=1.14 /
        # pos_open_rad=0.0）描述的不是任何真实行程，拿它算越位量比没有更糟。
        guard_rad = cfg.hold_runaway_rad
        guarded = gcfg.calibrated

        deadline = None if hold_s <= 0 else cfg.monotonic_fn() + hold_s
        frames_per_slice = max(1, int(round(cfg.hold_interval
                                            / cfg.frame_interval)))
        # 一帧一步。一片是 frames_per_slice 帧，只在片首走一步的话，20 N/s 在 0.2 s
        # 的片里就成了 4 N 一级的台阶，不是爬升。
        step_nm = cfg.force_ramp_n_s * cfg.frame_interval * UnitConversion.N_TO_NM
        cycles = 0
        st = g.get_state()
        pos = st.position_rad
        # 进入保力：从飞行中的力矩接着爬，交接才连续。飞行力矩已经越过设定值时
        # 直接从设定值起步（那一步是往下走到设定值，不是往上，不构成冲击）。
        tau_cmd = st.torque_nm if abs(st.torque_nm) < abs(target_nm) else target_nm

        # 护栏判在**第一帧之前**：已经在外面就直接失力。放进循环里判就晚了 ——
        # 那一片 frames_per_slice 帧的力矩已经发出去了，正是要拦住的东西。
        if guarded:
            out_rad = closing_travel_excursion_rad(gcfg, pos)
            if out_rad > guard_rad:
                log.warning("保力未开始：实测位置 %+.5f rad 已经越出闭合侧标定"
                            "限位 %+.5f rad 共 %.5f rad（> %.5f）—— 判定闭合侧"
                            "没有东西拦着它，撤力。",
                            pos, gcfg.pos_closed_rad, out_rad, guard_rad)
                self._release(pos)
                return HoldResult(ok=False, cycles=0, state=st, guarded=True)

        while deadline is None or cfg.monotonic_fn() < deadline:
            # 用上一次读到的位置当 q 下发（kp=0 时 q 不产生任何力，只是给下游
            # 一个不越位的指令），每帧把力矩朝设定值推一步，再回读状态查故障
            for _ in range(frames_per_slice):
                tau_cmd = _toward(tau_cmd, target_nm, step_nm)
                self._emit(pos, 0.0, tau_cmd, kp=0.0, kd=0.0)
            cycles += 1

            st = g.get_state()
            if st.error_code != 1:
                log.warning("保力中断：状态帧 err=%d"
                            "（1=使能中；0=被禁用；其他=故障）", st.error_code)
                return HoldResult(ok=False, cycles=cycles, state=st)
            pos = st.position_rad

            if guarded:
                out_rad = closing_travel_excursion_rad(gcfg, pos)
                if out_rad > guard_rad:
                    log.warning("保力中止：实测位置 %+.5f rad 越出闭合侧标定限位"
                                " %+.5f rad 共 %.5f rad（> %.5f）—— 判定闭合侧"
                                "没有东西拦着它，撤力。保力 %.1f N 到此为止。",
                                pos, gcfg.pos_closed_rad, out_rad, guard_rad,
                                force_n)
                    self._release(pos)
                    return HoldResult(ok=False, cycles=cycles, state=st,
                                      guarded=True)

            if progress is not None:
                progress(MoveProgress(
                    phase="hold",
                    i=cycles,
                    total_steps=0,
                    cmd_rad=pos,
                    pos_rad=pos,
                    delta_rad=0.0,
                    win_delta_rad=None,
                    torque_nm=st.torque_nm,
                    temperature_coil=st.temperature_coil,
                ))

        return HoldResult(ok=True, cycles=cycles, state=st)
