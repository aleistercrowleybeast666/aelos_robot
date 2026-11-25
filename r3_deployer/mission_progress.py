"""Present milestones from the robot's current leg."""

_CHECKPOINTS = {"outbound": (1, 2, 3, 4, 5), "return": (5, 6, 7, 1)}
_LEG_LABELS = {"outbound": "去程", "return": "返程"}
_LEG_TOTALS = {"outbound": 8, "return": 6}
_COMMAND0_TOTAL = 7
_COMMAND0_CORRIDOR_TAGS = (3, 4, 5)
_COMMAND0_CORRIDOR_TOTAL = _COMMAND0_TOTAL + 2 * len(_COMMAND0_CORRIDOR_TAGS)
_COMMAND0_DELIVERY_TOTAL = _COMMAND0_CORRIDOR_TOTAL + 2
_COMMAND2_RETURN_TAGS = _CHECKPOINTS["return"]
_COMMAND2_RETURN_CORRIDOR_TOTAL = 1 + 2 * len(_COMMAND2_RETURN_TAGS)
_COMMAND2_RETURN_TOTAL = _COMMAND2_RETURN_CORRIDOR_TOTAL + 3

_COMMAND0_PHASES = {
    "cmd0_prepare": (0, "准备原始物资试跑"),
    "cmd0_approach": (0, "接近中转区原始物资"),
    "cmd0_pickup": (1, "抓取原始物资"),
    "cmd0_carry": (2, "抱物前往障碍区入口"),
    "cmd0_turn": (3, "转向障碍区入口"),
    "cmd0_scan": (4, "入口扫码与纠偏"),
    "cmd0_entry_reacquire": (4, "入口持续重扫 Tag 1"),
    "cmd0_next_leg": (5, "抱物前往 Tag 2"),
    "cmd0_next_scan": (6, "试跑段动作完成，扫描 Tag 2"),
    "cmd0_next_scan_wait": (6, "持续重扫 Tag 2"),
}
_COMMAND0_CORRIDOR_PHASES = {
    "cmd0_corridor_leg": (0, "抱物前往 Tag {tag}"),
    "cmd0_corridor_scan": (1, "本段动作完成，扫描 Tag {tag}"),
    "cmd0_corridor_wait": (1, "持续重扫 Tag {tag}"),
}
_COMMAND0_DELIVERY_PHASES = {
    "cmd0_tag5_entry_wait": (_COMMAND0_CORRIDOR_TOTAL, "Tag 5 入口纠偏"),
    "cmd0_delivery_advance": (_COMMAND0_CORRIDOR_TOTAL, "抱物进入大本营"),
    "cmd0_delivery_release": (_COMMAND0_CORRIDOR_TOTAL + 1, "执行放物动作"),
    "cmd2_base_turn": (_COMMAND0_CORRIDOR_TOTAL + 1, "已放物起立，在大本营空载掉头"),
}
_COMMAND2_RETURN_PHASES = {
    "cmd2_return_leg": (0, "空载前往 Tag {tag}"),
    "cmd2_return_scan": (1, "本段动作完成，扫描 Tag {tag}"),
    "cmd2_return_wait": (1, "持续重扫 Tag {tag}"),
}
_COMMAND2_RETURN_STEPS = {
    "base_turn": (0, "放物后转向返程通道"),
    "transfer_advance": (_COMMAND2_RETURN_CORRIDOR_TOTAL, "前往中转区接驳位"),
    "transfer_approach": (_COMMAND2_RETURN_CORRIDOR_TOTAL, "前往中转区接驳位"),
    "transfer_turn": (_COMMAND2_RETURN_CORRIDOR_TOTAL + 1, "转向取物区"),
    "station_side": (_COMMAND2_RETURN_CORRIDOR_TOTAL + 2, "侧跨至接驳位"),
    "waiting_for_blocks": (_COMMAND2_RETURN_TOTAL, "在交接带外观察物资"),
}
_ROUTE_PHASES = {
    "pickup": ("outbound", 0, "识别已摆放的方块数量"),
    "start_wall_pickup": ("outbound", 0, "在中转区交接带对准方块"),
    "pickup_search": ("outbound", 0, "搜索待搬运物资"),
    "pickup_left": ("outbound", 0, "调整取物位置"),
    "pickup_right": ("outbound", 0, "调整取物位置"),
    "pickup_advance": ("outbound", 0, "接近待搬运物资"),
    "start_to_transfer": ("outbound", 1, "抱物前往中转区"),
    "transfer_to_entry": ("outbound", 1, "抱物前往通道入口"),
    "outbound_entry_scan": ("outbound", 2, "侧跨完成，正在确认入口 Tag 1"),
    "outbound_obstacles": ("outbound", 2, "穿越通道"),
    "base_approach": ("outbound", 7, "到大本营放物"),
    "base_to_return": ("return", 0, "从大本营空载返回"),
    "return_obstacles": ("return", 1, "沿返程通道行进"),
    "return_to_transfer": ("return", 5, "前往起始区靠墙工位"),
}
_WAITING_PHASES = {
    "cmd0_carry_wait": "持物航向复测中",
    "waiting_camera": "等待胸部相机",
    "waiting_sensors": "等待传感器",
    "recovering_heading": "恢复航向",
    "recovering_checkpoint": "寻找并确认路线 Tag",
}
_COMMAND0_COMPLETION = {
    "stage1_complete": (5, "入口扫码试跑已结束，保持持物。完整搬运待验收。"),
    "stage1_complete_blind": (5, "入口扫码试跑已结束，Tag 待确认，保持持物。完整搬运待验收。"),
    "stage1_partial_alignment": (5, "入口纠偏试跑已结束，完整搬运待验收。"),
    "stage1_alignment_blocked": (5, "已到入口纠偏阶段，完整搬运待验收。"),
    "stage2_checkpoint_reached": (7, "已扫到 Tag 2，持物试跑段完成。完整搬运待验收。"),
    "stage2_trial_unconfirmed": (5, "Tag 2 试跑待确认，请查看任务报告。"),
    "stage3_checkpoint_reached": (_COMMAND0_CORRIDOR_TOTAL, "已扫到 Tag 5，持物通道试跑完成。放物待验收。"),
    "stage3_trial_unconfirmed": (7, "通道试跑待确认，请查看任务报告。"),
}


def Command0_LastPassedTag(result):
    """Read the last route marker confirmed by the owner receipt."""
    tag = result.get("checkpoint")
    if type(tag) is not int:
        corridor = result.get("corridor_trial")
        tag = corridor.get("checkpoint") if isinstance(corridor, dict) else None
    return tag if type(tag) is int and tag in (2, *_COMMAND0_CORRIDOR_TAGS) else None


def Command2_LastReturnTag(result):
    """Read a confirmed return marker without using an expected or merely visible marker."""
    returned = result.get("return_trial")
    tag = returned.get("checkpoint") if isinstance(returned, dict) else None
    if type(tag) is not int:
        tag = result.get("last_passed_tag", result.get("checkpoint"))
    return tag if type(tag) is int and tag in _COMMAND2_RETURN_TAGS else None


def MissionDelivery_Label(command, outbound_only=False):
    if outbound_only and command in (2, "2"):
        return "完成首趟去程，实物落点待确认"
    return "搬运动作完成，实物落点待确认"


def MissionPickup_DoubleLoadWaitLabel(result):
    """Explain the same pickup wait in live progress and a saved receipt."""
    count = result.get("observed_count")
    if type(count) is not int or count < 2:
        return None
    if (result.get("reason_code") != "double_load_capability_unavailable"
            and result.get("reason") != "horizontal pair: complete double-load native capability unavailable"):
        return None
    reason = result.get("reason_zh")
    if isinstance(reason, str) and reason.strip():
        return reason
    return f"已识别 {count} 块，当前 MCU 双抱动作未就绪。可先摆一块试跑，双抱需登记动作包读回。"


def MissionDelivery_Tag5HoldLabel(result):
    """Describe a confirmed Tag 5 whose entry check did not approve forward travel."""
    if not isinstance(result, dict) or result.get("status") not in (
            "stage3_checkpoint_reached", "stage3_trial_unconfirmed", "stopped_no_further_auto_macros"):
        return None
    corridor = result.get("corridor_trial")
    if not isinstance(corridor, dict):
        events = result.get("events")
        if not isinstance(events, (list, tuple)):
            events = ()
        corridor = next((event.get("corridor_trial") for event in reversed(events)
                         if isinstance(event, dict) and isinstance(event.get("corridor_trial"), dict)), {})
    checkpoint = result.get("checkpoint")
    if type(checkpoint) is not int:
        checkpoint = corridor.get("checkpoint")
    if (result.get("release_protocol_completed") is True or corridor.get("release_protocol_completed") is True
            or checkpoint != 5):
        return None
    delivery = corridor.get("delivery")
    alignment = delivery.get("entry_lateral_alignment") if isinstance(delivery, dict) else None
    if not isinstance(alignment, dict):
        return None
    observations = alignment.get("observations")
    last = (observations[-1] if isinstance(observations, list) and observations
            and isinstance(observations[-1], dict) else {})
    if last.get("continued_after_expected_tag_occlusion") is True:
        return None
    ready = alignment.get("ready_for_forward")
    if ready is None:
        ready = last.get("ready_for_forward")
    if ready is False:
        text = "已扫到 Tag 5，入口纠偏尚未完成，放物尚未执行"
        if result.get("reason") in ("local task cancelled; no automatic resume",
                                    "manual takeover/cancel; no automatic resume"):
            return "已取消 · " + text
        return text
    return None


class MissionProgress:
    """Track confirmed route stages and display one direction at a time."""

    def __init__(self):
        self.reset()

    def reset(self, command=None):
        """Clear previous progress when a route starts or the target changes."""
        self._command = self._route_command(command)
        self._leg = None
        self._completed = None
        self._command0_total = _COMMAND0_TOTAL
        self._outbound_only = False
        self._shared_outbound = False
        self._return_trial = False
        self._active = False
        self._finished = False
        self.value = None if self._command is not None else 0
        self.text = "等待机器人报告任务阶段" if self._command is not None else "等待自动任务"

    @property
    def command(self):
        """Return the reported command when the current mission identifies it."""
        return self._command

    def observe(self, payload):
        """Apply an active owner's latest mission snapshot."""
        if not isinstance(payload, dict) or payload.get("ok") is False:
            return
        if payload.get("task_active") is False:
            self._finished = self._finished or self._active
            self._active = False
            return
        if payload.get("task_active") is not True:
            return
        if self._finished:
            if self._active:
                return  # A status request sent before completion can arrive late.
            self.reset()
        self._active = True

        progress = payload.get("mission_progress")
        if not isinstance(progress, dict) or not isinstance(progress.get("phase"), str):
            if self._completed is None:
                self._show(self._leg, None, "等待机器人报告任务阶段")
            return

        phase = progress["phase"]
        direction = progress.get("direction")
        if direction not in ("outbound", "return"):
            direction = self._leg
        reported = self._route_command(progress.get("command", payload.get("command")))
        if reported is not None:
            self._command = reported
        elif direction == "return":
            self._command = "2"
        if self._command in ("1", "2") and progress.get("outbound_only") is True:
            self._outbound_only = True
        if self._command in ("1", "2") and progress.get("shared_outbound") is True:
            self._shared_outbound = True
        shared_phase = (phase in _COMMAND0_PHASES or phase in _COMMAND0_CORRIDOR_PHASES
                        or phase in _COMMAND0_DELIVERY_PHASES)
        if shared_phase and not (self._outbound_only or self._shared_outbound):
            self._command = "0"
        if progress.get("delivery_planned") is True or phase in _COMMAND0_DELIVERY_PHASES:
            self._command0_total = _COMMAND0_DELIVERY_TOTAL
        elif self._uses_shared_route():
            if progress.get("trial_target_tag") == _COMMAND0_CORRIDOR_TAGS[-1] or phase in _COMMAND0_CORRIDOR_PHASES:
                self._command0_total = max(self._command0_total, _COMMAND0_CORRIDOR_TOTAL)
        checkpoint_note = progress.get("checkpoint_not_seen")
        checkpoint_text = None
        if isinstance(checkpoint_note, dict) and type(checkpoint_note.get("tag_id")) is int:
            if checkpoint_note.get("policy") == "continue_estimated":
                checkpoint_text = f"未扫到 Tag {checkpoint_note['tag_id']}，按预定路线继续"
            elif checkpoint_note.get("policy") == "hold_load_for_placement":
                checkpoint_text = f"未扫到 Tag {checkpoint_note['tag_id']}，保持持物，等待确认放物区"
        if phase in _COMMAND0_PHASES:
            completed, label = _COMMAND0_PHASES[phase]
            if phase == "cmd0_entry_reacquire" and isinstance(progress.get("reason_zh"), str):
                label = progress["reason_zh"].strip() or label
            self._show("outbound", completed, checkpoint_text or label)
        elif phase in _COMMAND0_CORRIDOR_PHASES:
            tag = progress.get("expected_tag")
            if type(tag) is int and tag in _COMMAND0_CORRIDOR_TAGS:
                offset, label = _COMMAND0_CORRIDOR_PHASES[phase]
                completed = _COMMAND0_TOTAL + 2 * _COMMAND0_CORRIDOR_TAGS.index(tag) + offset
                label = label.format(tag=tag)
                if progress.get("stationary_scan") is True:
                    label = (f"原地扫描 Tag {tag}" if phase == "cmd0_corridor_scan"
                             else f"原地重扫 Tag {tag}")
                last_seen = progress.get("last_passed_tag", progress.get("last_seen_tag"))
                if type(last_seen) is int and 2 <= last_seen < tag:
                    label = f"已扫到 Tag {last_seen}，{label}"
                self._show("outbound", completed, checkpoint_text or label)
        elif phase in _COMMAND0_DELIVERY_PHASES:
            completed, label = _COMMAND0_DELIVERY_PHASES[phase]
            if phase == "cmd0_delivery_advance" and progress.get("tag5_detected") is True:
                label = "已扫到 Tag 5，抱物进入大本营"
            if phase == "cmd0_tag5_entry_wait" and isinstance(progress.get("reason_zh"), str):
                reason = progress["reason_zh"].strip()
                if reason and reason != label:
                    label = f"{label} · {reason}"
            self._show("outbound", completed, label)
        elif (phase in _COMMAND2_RETURN_PHASES or phase == "cmd2_return_station"
              or (phase == "waiting_blocks" and direction == "return")):
            self._command = "2"
            self._return_trial = True
            self._show_return_trial(phase, progress)
        elif phase in _ROUTE_PHASES:
            leg, completed, label = _ROUTE_PHASES[phase]
            if leg == "return" or phase == "start_wall_pickup":
                self._command = "2"
            if phase == "start_wall_pickup":
                # This phase follows the return even when polling missed it.
                self._leg = None
                self._completed = None
            if phase == "pickup":
                label = MissionPickup_DoubleLoadWaitLabel(progress) or label
                if (progress.get("reason_code") == "staged_block_layout_ambiguous"
                        and isinstance(progress.get("reason_zh"), str)):
                    label = progress["reason_zh"]
            self._show(leg, completed, label)
        elif phase == "checkpoint_passed":
            tag = progress.get("accepted_tag")
            completed = self._checkpoint(direction, tag)
            if completed is not None:
                self._show(direction, completed, f"已确认 Tag {tag}")
        else:
            label = _WAITING_PHASES.get(phase, "等待机器人报告任务阶段")
            found = (self._checkpoint(direction, progress.get("last_passed_tag"))
                     if phase == "recovering_checkpoint" else None)
            self._show(direction, found, label)

    def finish(self, result):
        """Keep the observed progress and describe the terminal receipt."""
        self._finished = True
        self._active = True
        result = result if isinstance(result, dict) else {}
        status = result.get("status")
        reported = self._route_command(result.get("command"))
        command = self._command or reported
        if result.get("outbound_only") is True and (reported or command) in ("1", "2"):
            command = reported or command
            self._outbound_only = True
        shared = command == "0" or self._outbound_only or self._shared_outbound
        tag5_hold = MissionDelivery_Tag5HoldLabel(result)
        if tag5_hold:
            self._command = command or "0"
            self._shared_outbound = self._command in ("1", "2")
            self._command0_total = _COMMAND0_DELIVERY_TOTAL
            self._show("outbound", _COMMAND0_CORRIDOR_TOTAL, tag5_hold)
        elif status == "return_trial_unconfirmed":
            self._command = "2"
            self._return_trial = True
            tag = Command2_LastReturnTag(result)
            label = "返程试跑待确认，请查看任务报告。"
            completed = None
            if tag is not None:
                completed = self._return_checkpoint_completion(tag)
                label = f"已扫到 Tag {tag}，{label}"
            self._show("return", completed, label)
        elif shared and status == "task_complete" and result.get("ok") is True:
            self._command = command
            self._command0_total = _COMMAND0_DELIVERY_TOTAL
            turn = result.get("base_turn_trial")
            label = ("放物、起立和掉头动作完成，停在大本营"
                     if isinstance(turn, dict) and turn.get("protocol_completed") is True
                     else MissionDelivery_Label(command, self._outbound_only))
            self._show("outbound", _COMMAND0_DELIVERY_TOTAL, label)
        elif shared and isinstance(status, str) and status in _COMMAND0_COMPLETION:
            self._command = command
            completed, label = _COMMAND0_COMPLETION[status]
            if status == "stage2_checkpoint_reached":
                self._command0_total = _COMMAND0_TOTAL
            elif status == "stage3_checkpoint_reached":
                self._command0_total = _COMMAND0_CORRIDOR_TOTAL
            elif status == "stage3_trial_unconfirmed":
                self._command0_total = max(self._command0_total, _COMMAND0_CORRIDOR_TOTAL)
                tag = Command0_LastPassedTag(result)
                if tag is not None:
                    completed = _COMMAND0_TOTAL + 2 * (tag - 2)
                    label = f"已扫到 Tag {tag}，{label}"
            self._show("outbound", completed, label)
        elif result.get("ok") is True and status == "task_complete":
            self._show("outbound", self._total("outbound"), "单程搬运完成")
        else:
            if status == "transfer_boundary_complete":
                label = "已到预设试跑边界，完整搬运待验收。"
            elif status == "cancelled_before_dispatch":
                label = "任务尚未发出，已取消"
            elif status == "stopped_no_further_auto_macros":
                label = "自动任务余链已停止"
            elif status == "start_unconfirmed":
                label = "任务已接收，结果待确认" if result.get("accepted") is True else "任务结果待确认"
            elif result.get("ok") is False:
                label = "任务未完成"
            else:
                label = "等待确认任务结果"
            self._show(self._leg, None, label)
            if self.value is None:
                self.value = 0

    def _uses_shared_route(self):
        return self._command == "0" or self._outbound_only or self._shared_outbound

    def _total(self, leg):
        if leg == "return" and self._return_trial:
            return _COMMAND2_RETURN_TOTAL
        if leg == "outbound" and self._uses_shared_route():
            return self._command0_total
        return _LEG_TOTALS.get(leg, _LEG_TOTALS["outbound"])

    def _show_return_trial(self, phase, progress):
        step = progress.get("return_step")
        if phase == "waiting_blocks" or phase == "cmd2_return_station" and not step:
            step = "waiting_for_blocks"
        if isinstance(step, str) and step in _COMMAND2_RETURN_STEPS:
            completed, label = _COMMAND2_RETURN_STEPS[step]
            if step in ("transfer_advance", "transfer_approach") and progress.get("last_passed_tag") == 1:
                label = "已扫到 Tag 1，" + label
        else:
            tag = progress.get("expected_tag")
            if type(tag) is not int or tag not in _COMMAND2_RETURN_TAGS:
                self._show("return", None, "等待机器人报告返程阶段")
                return
            offset, pattern = _COMMAND2_RETURN_PHASES[phase]
            completed = 1 + 2 * _COMMAND2_RETURN_TAGS.index(tag) + offset
            label = pattern.format(tag=tag)
            last_tag = progress.get("last_passed_tag")
            if (type(last_tag) is int and last_tag in _COMMAND2_RETURN_TAGS
                    and _COMMAND2_RETURN_TAGS.index(last_tag) < _COMMAND2_RETURN_TAGS.index(tag)):
                label = f"已扫到 Tag {last_tag}，{label}"
        self._show("return", completed, label)

    @staticmethod
    def _return_checkpoint_completion(tag):
        return 3 + 2 * _COMMAND2_RETURN_TAGS.index(tag)

    def _show(self, leg, completed, label):
        previous = self.value if leg == self._leg else None
        if leg != self._leg:
            self._leg = leg
            self._completed = None
        if completed is not None:
            self._completed = max(completed, self._completed or 0)
        prefix = _LEG_LABELS.get(leg, "单程")
        self.text = f"{prefix} · {label}"
        self.value = (100 * self._completed // self._total(leg)
                      if self._completed is not None else None)
        if self._uses_shared_route() and self.value is not None and previous is not None:
            self.value = max(previous, self.value)  # Older owners may announce the target late.

    @staticmethod
    def _route_command(value):
        if type(value) in (int, str):
            command = str(value)
            if command in ("0", "1", "2"):
                return command
        return None

    @staticmethod
    def _checkpoint(leg, tag):
        route = _CHECKPOINTS.get(leg, ())
        if type(tag) is int and tag in route:
            before_tags = 2 if leg == "outbound" else 1
            return before_tags + route.index(tag) + 1
        return None
