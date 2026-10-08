"""LiteGrip 动作层 —— open / close / grasp / zero / enable / disable。

高层 ``LiteGrip`` 的 ``open()`` / ``close()`` / ``grasp()`` 等方法直接转发到
这里的 :class:`GripperActions`。单独成模块是为了让「怎么动」这套逻辑只写一遍：
ROS 2 桥接、RPC 服务、产品代码都能直接调 :attr:`LiteGrip.actions`，而不用各自
重写一遍斜坡和堵转判据。

为什么不用 ``goto_rad()`` / ``control_mit_stream()``
---------------------------------------------------
``control_mit_stream`` 在整段时长里反复下发**同一个** ``q_target``：伺服几十
毫秒就贴上去，剩下时间空转 —— 慢速时表现为一顿一停。这里的做法和 SDK 自己的
``move_at_speed`` 一样，按固定帧间隔推进一条线性斜坡，并给 ``dq_target`` 速度
前馈，所以是匀速连续运动。

指令领先实测位置的部分由领先上限封顶，分两档：行进段用
:attr:`MotionConfig.max_lead_mm`（大领先量破静摩擦），距限位
:attr:`MotionConfig.press_zone_mm` 之内切到
:attr:`MotionConfig.stop_lead_mm`（压紧段轻压）。不封顶的话，被挡住时误差会
一直累积、力矩顶到危险值；封顶后静摩擦靠满额力矩破，力矩却始终有界（约
``kp × lead_cap_rad``）。也不能改成「相对实测加一块」——那样一旦夹住，指令跟
着实测冻结，误差永远涨不上去，会误判堵转。

``open()`` / ``close()`` 的目标是**越过**标定限位一点（
:attr:`MotionConfig.press_overshoot`），靠堵转停在物理限位上，终点不依赖标定
精度。``grasp()`` 的闭合段仍停在限位内侧（:attr:`MotionConfig.margin`）——
夹取要停在工件上，不能压向空载限位。

堵转判据是软件侧的位置增量判据（电机本身没有堵转保护）：每
:attr:`MotionConfig.sample_interval` 采一次位置，连续
:attr:`MotionConfig.stall_cycles` 次采样的**窗口净位移**小于阈值即判堵转。
阈值 = ``max(stall_delta, stall_ratio × 窗口内本该走的距离)``。不看单点，因为
闭合侧有约 0.010 rad 的机械死区，慢速粘滑时单点忽大忽小。斜坡走完的保压段
不判（那时夹爪本来就该不动）。

保力段（``grasp`` 的第二段）反过来：只下发前馈力矩，不给位置/速度增益。
力控要的是力，带增益就会随夹爪的位移衰减，详见
:meth:`GripperActions._hold_force`。

本模块**不 print**：进度通过 ``progress`` 回调交给调用方（CLI 打印、ROS 节点
记日志、RPC 服务转发都行）。
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Optional, Tuple

from .constants import UnitConversion
from .exceptions import CommandError, LiteGripError
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
    margin: float = 0.05                # 距标定限位留下的行程余量比例（仅 grasp）
    frame_interval: float = 0.005       # 200 Hz 斜坡帧间隔 s
    sample_interval: float = 0.05       # 20 Hz 堵转采样间隔 s
    settle_s: float = 0.3               # 斜坡后原地保目标时长 s（不判堵转）
    reach_tol: float = 0.02             # 到位容差 rad（闭合侧死区 0.0103）

    # ── open/close 顶限位压紧 ──────────────────────────────────────────
    press_overshoot: float = 0.05       # 指令越过标定限位的行程比例
    press_zone_mm: float = 2.0          # 距限位这么近就切到 stop_lead_mm
    stop_lead_mm: float = 0.7           # 压紧段领先上限 mm（≈ kp × 上限）
    stop_tol: float = 0.02              # 停稳位置距限位多近算「顶在限位上」rad

    # ── 堵转判据 ───────────────────────────────────────────────────────
    stall_cycles: int = 5               # 窗口采样点数
    stall_ratio: float = 0.2            # 窗口净位移 / 本该走的距离
    stall_delta: float = 0.0015         # 阈值下限 rad

    # ── 力矩 / 指令上限 ────────────────────────────────────────────────
    max_lead_mm: float = 4.0            # 行进段领先上限 mm（≈ kp × 上限）

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
    hold_interval: float = 0.2          # 保力分片时长 s
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
    """open / close 的结果。

    ``ok`` 是「这次动作算不算成功」，``__bool__`` 用它。注意含义随目标而变：

    - ``open()`` / ``close()``：成功 = **顶到机械限位堵转**，所以
      ``ok=True`` 时 ``stalled=True`` 而 ``reached`` 基本为 ``False``。
      半路被工件挡住也算堵转，但离标定限位很远，``ok=False``。
    - ``grasp()`` 的闭合段：成功 = 走到空载目标且没堵转，即
      ``reached and not stalled``，与旧行为一致。
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

    与 :func:`limit_target` 相反 —— 目标是「压过去」，让夹爪顶着机械限位堵转，
    终点由物理限位决定，不依赖标定精度。压紧段的领先上限由
    :attr:`MotionConfig.stop_lead_mm` 收窄，所以压紧力矩 ≈
    ``kp × stop_lead_mm / rad_to_mm``，不会一路顶到 ``kp × 越位量``。

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
    留出一段余量（现场口径：机械行程 87 mm，工作只用到 80 mm，开口端留 7 mm）。
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


# ═══════════════════════════════════════════════════════════════════════════
# 动作层
# ═══════════════════════════════════════════════════════════════════════════

class GripperActions:
    """夹爪的六个动作：``open`` / ``close`` / ``grasp`` / ``zero`` /
    ``enable`` / ``disable``。

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

        默认顶到张开侧机械限位堵转（压紧段轻压，不会撞），``ok=True`` 时
        ``stalled=True``、``reached`` 基本为 ``False``。

        但若配置带了**工作行程**（:attr:`GripperConfig.work_stroke_mm` 且小于
        机械行程），只走到那里就停 —— 开口端留出余量，不再压机械限位。这时是
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
        """全合：直接顶到闭合侧机械限位堵转（压紧段轻压，不会撞）。

        ``ok=True`` 时 ``stalled=True``、``reached`` 基本为 ``False`` ——
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
        """夹取：先闭合到堵转（= 夹住工件），再持续输出 ``force_n`` 大小的力。

        Args:
            force_n: 夹持力 N，``None`` = 用 :attr:`MotionConfig.force_n`。
                按 SDK 近似换算 1 N = 0.1 Nm。
            hold_s: 保力时长 s。``0`` = 一直保到出错或 Ctrl+C。
            progress: 进度回调。

        Returns:
            :class:`GraspResult`。夹住工件时 ``stalled=True`` 且
            ``reached=False``（压不到空载目标位置是正常的）。
        """
        cfg = self.config
        force = cfg.force_n if force_n is None else force_n

        move = self._move_to_limit(
            "close", cfg.grasp_speed_mm_s, progress=progress)
        ok, cycles, st = self._hold_force(force, hold_s, progress=progress)
        return GraspResult(
            ok=ok,
            reached=move.reached,
            stalled=move.stalled,
            state=st if st is not None else move.state,
            target_rad=move.target_rad,
            force_n=force,
            cycles=cycles,
        )

    def zero(self) -> CalibrationData:
        """完整标定：探闭合 + 张开两个限位，算出行程与换算系数，并存盘。

        过程中夹爪会主动顶住两端机械限位（低刚度探测）。确保行程内无物。
        探测自带两道护栏：指令领先量不超过 ``calib_step_rad``，且 ``|tau|`` 一到
        ``calib_tau_limit`` 立即停止推进 —— 顶住限位时结构让位（背隙/弹性变形）
        会让位置读数一直在动，只靠「位置不再变化」是停不下来的。

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
              kp: Optional[float] = None, kd: Optional[float] = None) -> None:
        """下发一帧 MIT 并等待一帧的时间。"""
        cfg = self.config
        g = self._g
        sent = g.send_mit_frame(
            q,
            g.config.kp if kp is None else kp,
            g.config.kd if kd is None else kd,
            dq=dq,
            tau=tau,
        )
        if not sent:
            raise CommandError("MIT 帧下发失败（未连接或未使能）")
        cfg.sleep_fn(cfg.frame_interval)

    def _move_to_limit(
        self,
        toward: str,
        speed_mm_s: float,
        *,
        press: bool = False,
        target_rad: Optional[float] = None,
        progress: Optional[Callable[[MoveProgress], None]] = None,
    ) -> MoveResult:
        """走到一端的限位：整段是一条 frame_interval 一格的连续斜坡（MIT 帧 +
        速度前馈），边走边按位置判堵转。

        ``press=False``：目标是「限位内侧留 :attr:`MotionConfig.margin`」，
        到位即成功（``grasp`` 的闭合段用这个）。
        ``press=True``：目标是「越过限位 :attr:`MotionConfig.press_overshoot`」，
        靠堵转停在物理限位上；距限位 :attr:`MotionConfig.press_zone_mm` 之内
        领先上限切到 :attr:`MotionConfig.stop_lead_mm`，压紧力矩有界。

        ``target_rad``：显式目标（如 :func:`work_limit_target` 算出的工作行程
        点）。给了它就走**普通定位**语义（``press`` 视为 False），``limit`` 取该
        方向的标定限位用于上报。

        返回 :class:`MoveResult`。
        """
        cfg = self.config
        g = self._g
        g._check_enabled()
        gcfg = g.config

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
        speed_rad_s = speed_mm_s / gcfg.rad_to_mm

        interval = cfg.frame_interval
        ramp_s = dist_mm / speed_mm_s if speed_mm_s > 0 else 0.0
        ramp_steps = max(1, int(round(ramp_s / interval)))
        settle_steps = max(1, int(round(cfg.settle_s / interval)))
        total_steps = ramp_steps + settle_steps

        sample_every = max(1, int(round(cfg.sample_interval / interval)))
        win = max(cfg.stall_cycles, 1)
        win_s = win * sample_every * interval
        win_expect_rad = speed_rad_s * win_s
        win_thresh_rad = max(cfg.stall_delta, cfg.stall_ratio * win_expect_rad)

        # 两档领先上限：行进段用 max_lead_mm 破静摩擦，贴近限位后收窄到
        # stop_lead_mm，压紧力矩 ≈ kp × stop_lead_mm。下限都是一帧的位移，
        # 免得斜坡自己那一格被切掉。
        min_cap_rad = speed_rad_s * interval
        travel_cap_rad = max(cfg.max_lead_mm / gcfg.rad_to_mm, min_cap_rad)
        stop_cap_rad = max(cfg.stop_lead_mm / gcfg.rad_to_mm, min_cap_rad)
        press_zone_rad = cfg.press_zone_mm / gcfg.rad_to_mm

        log.info("%s %.4f → %.4f rad（%.1f mm/s，%.1f mm，%d+%d 帧，"
                 "堵转阈值 %.5f rad，领先上限 %.5f/%.5f rad%s）",
                 "闭合" if toward == "close" else "张开",
                 before.position_rad, target, speed_mm_s, dist_mm,
                 ramp_steps, settle_steps, win_thresh_rad,
                 travel_cap_rad, stop_cap_rad,
                 "，顶限位" if press else "")

        hist = [before.position_rad]
        stalled = False
        protected = False
        over_torque = 0                          # 连续过阈的采样点数
        prev_sample_pos = before.position_rad
        st = before
        last_cmd = before.position_rad
        last_i = 0

        for i in range(1, total_steps + 1):
            st = g.get_state(wait=False)
            pos = st.position_rad
            if i <= ramp_steps:
                q_sched = before.position_rad + dist_rad * (i / ramp_steps)
                dq = sign * speed_rad_s
            else:
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
            last_cmd = cmd
            self._emit(cmd, dq, 0.0)
            last_i = i

            if i % sample_every and i != total_steps:
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
                    total_steps=total_steps,
                    cmd_rad=cmd,
                    pos_rad=pos,
                    delta_rad=hist[-1] - hist[-2],
                    win_delta_rad=win_delta,
                    torque_nm=st.torque_nm,
                    temperature_coil=st.temperature_coil,
                ))
            # 保压段 (i > ramp_steps) 本来就该不动，所以默认不判堵转。
            # 但 press=True 时保压段的指令在限位外侧，夹爪本来就该顶着不动 ——
            # 那里的「不动」正是我们要的堵转。而且越位量（5% 行程）往往短于一个
            # 采样窗口，只在斜坡段判会永远判不到，动作得白跑完保压段。
            if (win_delta is not None and (press or i <= ramp_steps)
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
            # 触发即失力：连发 kp=kd=tau=0 的帧，让夹爪能被手掰动，而不是继续
            # 压着。q 用当前读数 —— 零增益下 q 不产生任何力，只是给个不越位的
            # 指令，免得下游把它当一次正常定位。
            release_frames = max(
                1, int(round(cfg.stop_release_s / cfg.frame_interval)))
            for _ in range(release_frames):
                self._emit(pos, 0.0, 0.0, kp=0.0, kd=0.0)

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
    ) -> Tuple[bool, int, Optional[GripperState]]:
        """持续输出夹持力：整段只下发前馈力矩。

        每片 :attr:`MotionConfig.hold_interval` 下发同一个 ``tau = force_n × 0.1``
        Nm，回读一次状态查故障。帧里 ``kp=kd=0`` —— 保力要的是**力**，而 MIT
        律里 ``kp × (q - 实测位置)`` 与 ``kd × (0 - 实测速度)`` 都随夹爪的位置
        和速度变化：工件在设定力下让位（或闭合侧约 0.010 rad 的粘滑死区走一
        格），实测位置就往前挪，``kp × 位移`` 立刻从前馈里扣掉一截，读数表现
        为「先夹到设定力，过一会儿掉到某个更小的值」。上一版锚在 200 ms 前的
        位置读数上、``kp=150``，工件以 3 mm/s 让位就能把 20 N 读成 5 N。所以
        **不要**为了「顶得更硬」把增益加回来；:attr:`MotionConfig.hold_kp` /
        :attr:`MotionConfig.hold_kd` 已废弃，设了也不生效。

        代价：零增益下夹爪可以被外力推动，夹到空载时也会一路顶到机械限位
        （和 ``close()`` 的压紧段一样，只是力矩小得多）。

        ``hold_s <= 0`` 表示不限时长（直到出错或 Ctrl+C）。

        Returns:
            ``(是否正常结束, 保力片数, 最后一次状态)``。
        """
        cfg = self.config
        g = self._g
        # 夹紧方向的力矩符号随安装方向翻转：正装时 rad 增大是闭合，
        # 反装时反过来。力矩大小不变，只是要让前馈往「夹」而不是「撑」。
        tau_nm = g.config.close_sign * force_n * UnitConversion.N_TO_NM

        deadline = None if hold_s <= 0 else cfg.monotonic_fn() + hold_s
        frames_per_slice = max(1, int(round(cfg.hold_interval
                                            / cfg.frame_interval)))
        cycles = 0
        st = g.get_state()
        pos = st.position_rad

        while deadline is None or cfg.monotonic_fn() < deadline:
            # 用上一次读到的位置当 q 下发（kp=0 时 q 不产生任何力，只是给下游
            # 一个不越位的指令），再回读状态查故障
            for _ in range(frames_per_slice):
                self._emit(pos, 0.0, tau_nm, kp=0.0, kd=0.0)
            cycles += 1

            st = g.get_state()
            if st.error_code != 1:
                log.warning("保力中断：状态帧 err=%d"
                            "（1=使能中；0=被禁用；其他=故障）", st.error_code)
                return False, cycles, st
            pos = st.position_rad

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

        return True, cycles, st
