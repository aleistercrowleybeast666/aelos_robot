import unittest

from r3_deployer.mission_progress import MissionDelivery_Tag5HoldLabel, MissionProgress


def _active(phase, **detail):
    return {"ok": True, "task_active": True, "mission_progress": {"phase": phase, **detail}}


class MissionProgressTests(unittest.TestCase):
    def test_continuous_mission_resets_each_leg(self):
        progress = MissionProgress()
        progress.reset("2")
        progress.observe(_active("checkpoint_passed", direction="outbound", accepted_tag=5))
        self.assertEqual(progress.value, 87)
        progress.observe(_active("base_to_return"))
        self.assertEqual(progress.value, 0)
        self.assertIn("返程", progress.text)
        progress.observe(_active("checkpoint_passed", direction="return", accepted_tag=5))
        self.assertEqual(progress.value, 33)
        progress.observe(_active("return_to_transfer"))
        self.assertEqual(progress.value, 83)
        progress.observe(_active("start_wall_pickup"))
        self.assertEqual(progress.value, 0)
        self.assertIn("去程", progress.text)

    def test_checkpoint_recovery_only_uses_confirmed_position(self):
        progress = MissionProgress()
        progress.observe(_active("recovering_checkpoint", direction="outbound",
                                 expected_tag=4, seen_tags=[4, 5], last_passed_tag=3))
        compare = progress.value
        self.assertEqual(compare, 62)
        progress.observe(_active("waiting_camera", direction="outbound", expected_tag=5))
        progress.observe(_active("recovering_heading"))
        self.assertEqual(progress.value, compare)
        progress.observe(_active("checkpoint_passed", direction="outbound", expected_tag=4,
                                 last_passed_tag=3, accepted_tag=5))
        self.assertEqual(progress.value, 87)

    def test_command_zero_trial_has_remaining_route_after_entrance(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe({"task_active": True, "mission_progress": None})
        self.assertIsNone(progress.value)
        progress.observe(_active("cmd0_pickup"))
        progress.observe(_active("cmd0_scan"))
        self.assertEqual(progress.value, 57)
        progress.finish({"ok": True, "command": "0", "status": "stage1_complete"})
        self.assertEqual(progress.value, 71)
        self.assertIn("完整搬运待验收", progress.text)

    def test_command_zero_corridor_progress_uses_one_leg_and_preserves_confirmed_stage(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe(_active("cmd0_next_scan", trial_target_tag=5))
        self.assertEqual(progress.value, 46)
        progress.observe(_active("cmd0_corridor_leg", expected_tag=3, last_passed_tag=2))
        self.assertEqual(progress.value, 53)
        self.assertIn("已扫到 Tag 2", progress.text)
        self.assertIn("前往 Tag 3", progress.text)
        progress.observe(_active("cmd0_corridor_scan", expected_tag=3))
        self.assertEqual(progress.value, 61)
        progress.observe(_active("cmd0_corridor_wait", expected_tag=3, seen_tags=[3, 4]))
        self.assertEqual(progress.value, 61)
        self.assertIn("重扫 Tag 3", progress.text)
        progress.observe(_active("cmd0_corridor_leg", expected_tag=4, last_seen_tag=3))
        self.assertEqual(progress.value, 69)
        progress.finish({"ok": False, "status": "stage3_trial_unconfirmed"})
        self.assertEqual(progress.value, 69)
        self.assertIn("通道试跑待确认", progress.text)

    def test_command_zero_tag5_finish_and_older_tag2_finish(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe(_active("cmd0_next_scan"))
        self.assertEqual(progress.value, 85)
        progress.observe(_active("cmd0_corridor_leg", expected_tag=3))
        self.assertEqual(progress.value, 85)
        progress.finish({"ok": True, "status": "stage3_checkpoint_reached"})
        self.assertEqual(progress.value, 100)
        self.assertIn("Tag 5", progress.text)
        self.assertIn("放物待验收", progress.text)
        progress.reset("0")
        progress.finish({"ok": True, "status": "stage2_checkpoint_reached"})
        self.assertEqual(progress.value, 100)
        self.assertIn("Tag 2", progress.text)

    def test_command_zero_progress_keeps_delivery_pending_until_receipt(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe(_active("cmd0_next_scan", trial_target_tag=5, delivery_planned=True))
        self.assertEqual(progress.value, 40)
        progress.observe(_active("cmd0_corridor_leg", expected_tag=4, last_passed_tag=3))
        self.assertEqual(progress.value, 60)
        self.assertIn("已扫到 Tag 3，抱物前往 Tag 4", progress.text)
        progress.observe(_active("cmd0_corridor_scan", expected_tag=5, last_passed_tag=4, stationary_scan=True))
        self.assertIn("已扫到 Tag 4，原地扫描 Tag 5", progress.text)
        progress.observe(_active("cmd0_delivery_advance", last_passed_tag=5))
        self.assertEqual(progress.value, 86)
        self.assertNotIn("已扫到",progress.text)
        progress.observe(_active("cmd0_delivery_advance", last_passed_tag=5, tag5_detected=True))
        self.assertIn("已扫到 Tag 5，抱物进入大本营", progress.text)
        progress.observe(_active("cmd0_delivery_release", last_passed_tag=5))
        self.assertEqual(progress.value, 93)
        self.assertIn("执行放物动作", progress.text)
        progress.finish({"ok": True, "status": "task_complete", "stage": 4})
        self.assertEqual(progress.value, 100)
        self.assertIn("搬运动作完成，实物落点待确认", progress.text)

    def test_tag1_progress_shows_measured_sweep_and_stationary_recheck(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe(_active("cmd0_entry_reacquire", reason_zh="正扫入口两侧，当前相对起点 -18°。"))
        value = progress.value
        self.assertIn("正扫入口两侧",progress.text)
        progress.observe(_active("cmd0_entry_reacquire", reason_zh="检测器读到了 Tag 1，正在原地复核有效帧。"))
        self.assertEqual(progress.value,value)
        self.assertIn("原地复核有效帧",progress.text)

    def test_tag5_wait_stays_at_entry_progress_until_forward_approval(self):
        progress = MissionProgress()
        progress.reset("2")
        progress.observe(_active("cmd0_tag5_entry_wait", command="2", shared_outbound=True,
                                 reason_zh="预计横偏超出入口余量"))
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 86)
        self.assertEqual(progress.text, "去程 · Tag 5 入口纠偏 · 预计横偏超出入口余量")
        progress.observe(_active("cmd0_delivery_advance", command="2", shared_outbound=True))
        self.assertEqual(progress.value, 86)
        self.assertIn("抱物进入大本营", progress.text)

    def test_cancel_during_tag5_pose_wait_uses_latest_corridor_event(self):
        earlier = {"checkpoint": 5, "release_protocol_completed": True}
        latest = {"checkpoint": 5, "release_protocol_completed": False, "delivery": {
            "entry_lateral_alignment": {"ready_for_forward": False},
        }}
        result = {"ok": False, "command": "2", "status": "stopped_no_further_auto_macros",
                  "reason": "local task cancelled; no automatic resume",
                  "events": [{"corridor_trial": earlier}, {"corridor_trial": latest}]}
        progress = MissionProgress()
        progress.finish(result)
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 86)
        self.assertIn("已取消 · 已扫到 Tag 5，入口纠偏尚未完成，放物尚未执行", progress.text)
        self.assertIsNone(MissionDelivery_Tag5HoldLabel(dict(result, release_protocol_completed=True)))
        self.assertIsNone(MissionDelivery_Tag5HoldLabel(dict(result, events=[{"corridor_trial": earlier}])))

    def test_command_zero_incomplete_receipt_reports_last_confirmed_tag(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe(_active("cmd0_prepare", delivery_planned=True))
        progress.finish({"ok": False, "status": "stage3_trial_unconfirmed", "checkpoint": 5})
        self.assertEqual(progress.value, 86)
        self.assertIn("已扫到 Tag 5，通道试跑待确认", progress.text)

    def test_first_outbound_shared_phases_preserve_commands_one_and_two(self):
        for command in ("1", "2"):
            with self.subTest(command=command):
                progress = MissionProgress()
                progress.reset(command)
                details = {"command": command, "outbound_only": True, "delivery_planned": True}
                progress.observe(_active("pickup", **details))
                progress.observe(_active("cmd0_next_scan", **details))
                self.assertEqual(progress.command, command)
                self.assertEqual(progress.value, 40)
                progress.observe(_active("cmd0_corridor_leg", expected_tag=4, last_passed_tag=3, **details))
                self.assertEqual(progress.command, command)
                self.assertIn("已扫到 Tag 3", progress.text)
                progress.observe(_active("cmd0_delivery_release", **details))
                self.assertEqual(progress.command, command)
                self.assertEqual(progress.value, 93)
                progress.finish({"ok": True, "status": "task_complete", "command": command,
                                 "outbound_only": True, "return_started": False})
                self.assertEqual(progress.value, 100)
                ending = "完成首趟去程" if command == "2" else "搬运动作完成"
                self.assertIn(ending + "，实物落点待确认", progress.text)

    def test_first_outbound_completion_receipt_without_progress_poll(self):
        progress = MissionProgress()
        progress.finish({"ok": True, "status": "task_complete", "command": 2,
                         "outbound_only": True, "return_started": False})
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 100)
        self.assertIn("完成首趟去程", progress.text)

    def test_staged_triple_wait_displays_the_mcu_requirement_live(self):
        progress = MissionProgress()
        progress.reset("1")
        progress.observe(_active("pickup", command="1", observed_count=3,
                                 reason_code="double_load_capability_unavailable"))
        self.assertIn("已识别 3 块，当前 MCU 双抱动作未就绪。", progress.text)
        self.assertIn("可先摆一块试跑，双抱需登记动作包读回。", progress.text)

    def test_command_two_return_waits_at_station_and_resets_each_leg(self):
        progress = MissionProgress()
        progress.reset("2")
        outbound = {"command": "2", "shared_outbound": True, "outbound_only": False, "delivery_planned": True}
        progress.observe(_active("cmd0_delivery_release", **outbound))
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 93)
        progress.observe(_active("cmd2_return_leg", direction="return", return_step="base_turn"))
        self.assertEqual(progress.value, 0)
        self.assertEqual(progress.text, "返程 · 放物后转向返程通道")
        progress.observe(_active("cmd2_return_leg", direction="return", return_step="corridor", expected_tag=5))
        self.assertEqual(progress.value, 8)
        progress.observe(_active("cmd2_return_wait", direction="return", return_step="corridor",
                                 expected_tag=7, last_passed_tag=6))
        self.assertEqual(progress.value, 50)
        self.assertEqual(progress.text, "返程 · 已扫到 Tag 6，持续重扫 Tag 7")
        progress.observe(_active("cmd2_return_leg", direction="return", return_step="transfer_advance"))
        self.assertEqual(progress.value, 75)
        self.assertNotIn("已扫到",progress.text)
        progress.observe(_active("cmd2_return_leg", direction="return", return_step="transfer_advance", last_passed_tag=1))
        self.assertIn("已扫到 Tag 1", progress.text)
        progress.observe(_active("cmd2_return_leg", direction="return", return_step="station_side"))
        self.assertEqual(progress.value, 91)
        self.assertIn("侧跨至接驳位", progress.text)
        progress.observe(_active("cmd2_return_station", direction="return", return_step="waiting_for_blocks"))
        progress.observe(_active("waiting_blocks", command="2", direction="return"))
        self.assertEqual(progress.value, 100)
        self.assertEqual(progress.text, "返程 · 在交接带外观察物资")
        progress.observe(_active("start_wall_pickup", **outbound))
        self.assertEqual(progress.value, 0)
        self.assertEqual(progress.text, "去程 · 在中转区交接带对准方块")
        progress.observe(_active("cmd0_next_scan", **outbound))
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 40)

    def test_command_two_next_pickup_resets_shared_scale_if_return_polls_are_missed(self):
        progress = MissionProgress()
        outbound = {"command": 2, "shared_outbound": True, "outbound_only": False}
        progress.observe(_active("cmd0_delivery_release", **outbound))
        self.assertEqual(progress.value, 93)
        progress.observe(_active("start_wall_pickup", **outbound))
        self.assertEqual(progress.value, 0)
        self.assertEqual(progress.command, "2")

    def test_command_two_return_receipt_uses_last_confirmed_marker_without_poll(self):
        progress = MissionProgress()
        progress.finish({"ok": False, "command": "2", "status": "return_trial_unconfirmed",
                         "checkpoint": 5, "return_trial": {"checkpoint": 7, "expected_tag": 1}})
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 58)
        self.assertEqual(progress.text, "返程 · 已扫到 Tag 7，返程试跑待确认，请查看任务报告。")

    def test_idle_and_trailing_active_probe_keep_terminal_receipt(self):
        progress = MissionProgress()
        progress.reset("1")
        progress.finish({"ok": True, "command": "1", "status": "task_complete"})
        receipt = progress.text
        progress.observe(_active("base_approach"))
        progress.observe({"task_active": False})
        self.assertEqual(progress.value, 100)
        self.assertEqual(progress.text, receipt)
        progress.observe(_active("return_obstacles"))
        self.assertEqual(progress.command, "2")
        self.assertIn("返程", progress.text)
        self.assertEqual(progress.value, 16)

    def test_next_pickup_resets_without_observed_return(self):
        progress = MissionProgress()
        progress.observe(_active("base_approach"))
        progress.observe(_active("start_wall_pickup"))
        self.assertEqual(progress.command, "2")
        self.assertEqual(progress.value, 0)

    def test_old_status_without_live_progress_and_cancellation(self):
        progress = MissionProgress()
        progress.reset("0")
        progress.observe({"task_active": True})
        self.assertIsNone(progress.value)
        progress.finish({"ok": False, "status": "cancelled_before_dispatch"})
        self.assertEqual(progress.value, 0)
        self.assertIn("已取消", progress.text)
        progress.reset()
        progress.observe({"task_active": True})
        self.assertIsNone(progress.value)


if __name__ == "__main__":
    unittest.main()
