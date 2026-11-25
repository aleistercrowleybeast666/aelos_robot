import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from r3_deployer import task_report
from r3_deployer.control_api import RobotControlUncertainError


class TaskReportTests(unittest.TestCase):
    def test_nominal_release_and_default_pair_reports_do_not_claim_a_visual_detection(self):
        result = {"ok":True, "pickup":{"default_pair_station":"tag1_handoff","observed_count":None,
                  "grip_count":2,"vision_reason":"block camera has no fresh post-settle stream"},
                  "return_trial":{"estimated_pickup_anchor_tag":1},
                  "next_segment":{"checkpoint_not_seen":{"tag_id":2,"policy":"continue_estimated",
                                                         "cause":"insufficient_frames"}},
                  "corridor_trial":{"detected":False,"release_protocol_completed":True,
                        "missing_checkpoints":[{"tag_id":5,"policy":"hold_load_for_placement",
                                                "cause":"camera_stream_unhealthy"}]}}
        text = "\n".join(task_report.TaskReport_Diagnostics(result))
        self.assertIn("返程接驳区未确认方块数量",text)
        self.assertIn("动作结束后未收到新的方块相机帧",text)
        self.assertIn("返程 Tag 1 未确认",text)
        self.assertIn("Tag 2 已解码，有效帧不足",text)
        self.assertIn("Tag 5 扫码流暂未恢复 · 已记录，按预定路线继续",text)
        self.assertNotIn("保持持物",text)
        self.assertNotIn("识别到 2 块",text)

    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.settings_path = Path(temporary.name) / "settings.json"
        credentials = patch.object(task_report, "settings_path", return_value=self.settings_path)
        credentials.start()
        self.addCleanup(credentials.stop)

    def save(self, outcome, password=None):
        return task_report.TaskReport_Save(outcome, target="192.0.2.35:22", command=0,
                                           after_update=True, password=password)

    def test_composite_receipt_retains_deployment_and_failure_events(self):
        path = self.save({
            "ok": False, "deployment": {"ok": True},
            "start": {"status": "stopped_no_further_auto_macros",
                      "events": [{"error": "remote output example-secret",
                                  "credential": "other-private-value"}]},
        }, "example-secret")
        raw = path.read_text(encoding="utf-8")
        report = json.loads(raw)
        self.assertTrue(report["after_update"])
        self.assertTrue(report["result"]["deployment"]["ok"])
        self.assertEqual(report["result"]["start"]["events"][0]["error"], "remote output [密码已隐藏]")
        self.assertNotIn("example-secret", raw)
        self.assertNotIn("other-private-value", raw)

    def test_uncertain_result_still_records_local_failure(self):
        path = self.save(RobotControlUncertainError("receipt lost"))
        report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["result"]["status"], "start_unconfirmed")
        self.assertEqual(report["result"]["reason"], "receipt lost")

    def test_tag2_boundary_shortening_is_reported_separately_from_tag1_compensation(self):
        result = {"ok":False,"entrance_trial":{
            "nominal_target_steps":10,"initial_estimated_steps":14,"planned_steps":12,
            "completed_steps":12,"next_stage_started":True,"reused_boundary_frames":True,
            "entry_boundary_observations":[{"after_steps":12,"previous_remaining_steps":2,
                "remaining_steps":0,"distance_m":.185,"heading_deg":-12.26}]}}
        lines = task_report.TaskReport_Diagnostics(result)
        self.assertIn("Tag 1 前后补偿 · 已合入直行计划 10 → 14 步",lines)
        self.assertIn("Tag 2 提前读码 · 第 12 步 · 剩余直行 2 → 0 步 · 距码约 18.5 cm · 航向 -12.26°",lines)
        self.assertIn("Tag 2 已复用当前动作结束后的有效帧",lines)

    def test_failed_atomic_publish_cleans_temporary_file(self):
        with patch.object(task_report.os, "replace", side_effect=PermissionError("publish denied")):
            with self.assertRaises(PermissionError):
                self.save({"event": "done", "ok": True})
        self.assertEqual(list((self.settings_path.parent / "task_reports").iterdir()), [])

    def test_older_failure_receipt_without_heading_uses_protocol_action(self):
        lines = task_report.TaskReport_Diagnostics({
            "ok": False, "observed_status": {"events": [
                None, {"stage_timing": {"stage": "protocol_action", "action": "carry_forward2",
                                       "protocol_completed": True}},
            ]},
        })
        self.assertEqual(lines, ("最近动作 · carry_forward2 · 收到完成回执",))
        self.assertEqual(task_report.TaskReport_Diagnostics({"ok": False, "events": None}), ())

    def test_magnetic_startup_failure_is_localized_without_changing_original_receipt(self):
        reason = (
            "micro trial stopped at completed macro boundary: fresh magnetic sample unavailable: "
            "{'status': 'skipped', 'reason': 'insufficient fresh magnetic receipt events'}"
            "; no remaining route, no reverse retry, no release; native immediate stop unavailable"
        )
        outcome = {
            "ok": False, "status": "stopped_no_further_auto_macros", "reason": reason,
            "events": [{"stage_timing": {"stage": "micro_capability"}},
                       {"stage_timing": {"stage": "sensor_preflight", "elapsed_s": 0.18079}}],
        }
        report = json.loads(self.save(outcome).read_text(encoding="utf-8"))
        self.assertEqual(report["result"], outcome)
        self.assertEqual(report["summary_zh"]["reason"],
                         "启动检查未通过 · 未收到足够的新鲜磁力计样本。未发出搬运动作。")
        outcome["events"].append({"stage_timing": {"stage": "protocol_action"}})
        self.assertEqual(task_report.TaskReport_Reason(outcome),
                         "自动任务中断 · 未收到足够的新鲜磁力计样本。未执行后续动作。")

    def test_cancelled_pickup_wait_shows_the_unavailable_double_load(self):
        result = {"ok": False, "status": "stopped_no_further_auto_macros",
                  "reason": "local task cancelled; no automatic resume",
                  "events": [{"pickup": {"status": "no_pickup", "observed_count": 3,
                                        "reason": "horizontal pair: complete double-load native capability unavailable"}}]}
        self.assertEqual(task_report.TaskReport_Reason(result), "自动任务已取消，后续动作未恢复")
        label = "已识别 3 块，当前 MCU 双抱动作未就绪。可先摆一块试跑，双抱需登记动作包读回。"
        self.assertIn(label, task_report.TaskReport_Diagnostics(result))
        result["events"].append({"pickup": {"status": "no_pickup",
                                            "reason_code": "staged_block_layout_ambiguous",
                                            "reason_zh": "看到了方块，正在复核。"}})
        self.assertIn(label, task_report.TaskReport_Diagnostics(result))
        result["events"].append({"pickup": {"status": "single_pickup_protocol_completed", "grip_count": 1,
                                             "observed_count": 1}})
        self.assertNotIn(label, task_report.TaskReport_Diagnostics(result))

    def test_heading_limit_keeps_original_measurements(self):
        reason = "heading error exceeds configured trial envelope: error=105.0 limit=60"
        self.assertEqual(task_report.TaskReport_Message(reason), "航向偏差 105.0° 超出试跑上限 60°")
        self.assertEqual(task_report.TaskReport_Message("unrecognized issue 105.0"), "unrecognized issue 105.0")

    def test_mcu_handshake_failure_reports_last_contact(self):
        result = {"ok": False, "reason": "MCU handshake missing/stale",
                  "diagnostic": {"mcu_last_contact_age_s": 3.751, "mcu_last_contact_source": "request"}}
        self.assertEqual(task_report.TaskReport_Reason(result), "树莓派未收到 MCU 的新握手")
        self.assertEqual(task_report.TaskReport_Diagnostics(result), ("MCU 通信 · 距上次收到消息（请求） 3.75 秒",))
        result.pop("diagnostic")
        self.assertEqual(task_report.TaskReport_Diagnostics(result), ())

    def test_action_timeout_keeps_action_and_duration(self):
        reason = ("operation carry_forward2 returned near timeout (3.900s / 4s); "
                  "completion uncertain; stop remaining macros")
        self.assertEqual(task_report.TaskReport_Message(reason),
                         "动作 carry_forward2 接近超时（耗时 3.900 秒，上限 4 秒），完成状态未确认，未执行后续动作")

    def test_new_chinese_receipt_reason_takes_precedence(self):
        result = {"ok": False, "reason": "original", "reason_zh": "启动检查未收到磁力计数据"}
        self.assertEqual(task_report.TaskReport_Reason(result), result["reason_zh"])

    def test_scan_log_localizes_status_and_reason(self):
        self.assertEqual(task_report.TaskReport_Message("AR扫码：recognized；no safe predicted visual correction"),
                         "AR扫码 · 已识别 · 当前没有预计能减小误差的视觉纠偏动作")

    def test_completed_route_logs_missed_checkpoints_and_compensating_step(self):
        result = {"ok": True, "corridor_trial": {
            "missing_checkpoints": [{"tag_id": 3, "policy": "continue_estimated"}],
            "legs": [{"from_tag": 2, "expected_id": 3, "lateral_heading_checks": [
                {"compensation_status": "applied", "compensation_action": "cmd0_carry_backward2", "drift_deg": -14}
            ]}],
        }}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "未扫到 Tag 3 · 已记录，按预定路线继续",
            "Tag 2 → 3 · 侧跨偏航 -14.00° · 纠偏完成后后退两步",
        ))
        check = result["corridor_trial"]["legs"][0]["lateral_heading_checks"][0]
        check["compensation_order"] = "before_heading_correction"
        self.assertEqual(task_report.TaskReport_Diagnostics(result)[-1],
                         "Tag 2 → 3 · 侧跨偏航 -14.00° · 先后退两步腾出空间，再复测航向")

    def test_small_left_wrong_way_is_localized(self):
        reason = "correction verification failed: wrong_direction_or_zero_change; no retry; {\"observed_change_deg\":4}"
        self.assertEqual(task_report.TaskReport_Message(reason), "转向复测未通过 · 实测转向与指令相反或未转动")

    def test_tag1_sweep_receipt_distinguishes_detector_and_accepted_frames(self):
        result = {"ok":False, "events":[{"terminal_tag_sweep":{"coverage_deg":[-17.,18.],
            "observations":[{"decoded_expected_frames":4,"accepted_expected_frames":0}]}}]}
        notes = task_report.TaskReport_Diagnostics(result)
        self.assertIn("Tag 1 搜码实测 · 相对起始朝向 -17° 至 +18°",notes)
        self.assertIn("Tag 1 最近一窗 · 检测器报出 4 帧 · 程序接受 0 帧",notes)

    def test_tag4_blocked_approach_reports_consistent_skew_and_recovery(self):
        leg = {"from_tag":3,"expected_id":4,"tag4_approach":[
            {"partial_advice":"outside_heading_or_lateral_range","distance_m":.418,
             "partial_pose":{"consistent":True,"latest_fresh":True,"samples":3,"heading_deg":-17.11}}]}
        result = {"ok":False,"corridor_trial":{"legs":[leg]}}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "Tag 4 进近 · 3 帧位姿一致 · 距码约 41.8 cm · 视觉航向 -17.11° · 位姿超出本组前进容差",
        ))
        leg["tag4_forward_disruption"] = {"reason_zh":"Tag 4 实测航向仍偏，先纠偏重扫。",
            "heading_error_deg":-17.11,"evidence_distance_m":.418,
            "visual_turn":{"action":"single_micro_right"},"reacquired":False}
        leg["tag4_partial_recovery"] = {"reason_zh":"按剩余计划短进并复扫。",
            "status":"completed","heading_deg":-16.42,"distance_m":.405,"completed_steps":2}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "Tag 4 进近恢复 · 距码约 40.5 cm · 视觉航向 -16.42° · 已前进 2 步 · 按剩余计划短进并复扫。",
            "Tag 4 实测航向仍偏，先纠偏重扫。 · 距码约 41.8 cm · 纠偏前 -17.11° · 已执行小右转 · 通道仍待确认",
        ))

    def test_tag4_reports_expired_single_frame_and_magnetic_continuation(self):
        leg = {"from_tag":3,"expected_id":4,"tag4_approach":[
            {"partial_advice":"outside_heading_or_lateral_range",
             "partial_pose":{"consistent":True,"latest_fresh":False,"samples":1,"heading_deg":-17.54}}]}
        report = {"ok":False,"corridor_trial":{"legs":[leg]}}
        partial = "Tag 4 进近 · 仅有 1 帧位姿，复扫时已过期"
        self.assertEqual(task_report.TaskReport_Diagnostics(report), (partial,))
        leg["tag4_magnetic_fallback"] = {"reason_zh":"按磁航向沿原路线继续。",
            "heading_deg":-4.6,"completed_steps":4,"planned_steps":12,"next_steps":2}
        fallback = "Tag 4 磁航向续行 · 磁航向余差 -4.60° · 直行已完成 4/12 步 · 下组计划 2 步 · 按磁航向沿原路线继续。"
        self.assertEqual(task_report.TaskReport_Diagnostics(report), (partial, fallback))
        leg["tag4_magnetic_fallback"] = [leg["tag4_magnetic_fallback"], {"reason_zh":"等待复测。"}]
        self.assertEqual(task_report.TaskReport_Diagnostics(report), (
            partial, fallback, "Tag 4 磁航向续行 · 等待复测。",
        ))

    def test_command1_retreat_reports_native_group_and_yaw_before_and_after(self):
        result = {"ok": True, "pickup_exit": {"retreat": {
            "planned_steps": 4, "continuous": True, "heading_status": "corrected_once",
            "completed_actions": ["cmd1_carry_backward4"], "heading_before_correction_deg": 16.,
            "residual_deg": 5.,
        }}}
        self.assertIn("连续后退 4 步 · 纠偏前 +16.00° · 余差 +5.00°",
                      task_report.TaskReport_Diagnostics(result))

    def test_tag5_report_displays_ground_pose_passage_and_completed_trim(self):
        result = {"ok": False, "events": [{"corridor_trial": {"legs": [{
            "from_tag": 4, "expected_id": 5, "tag5_midpoint": {
                "decision": "replace_remaining_strafes", "plan": {
                    "distance_m": .24, "heading_deg": -6, "lateral_error_m": .08,
                },
            },
        }], "delivery": {"entry_lateral_alignment": {
            "observations": [
                {"heading_deg": -6, "lateral_error_m": .08, "ready_for_forward": False,
                 "reason": "Tag 5 projected path exceeds the lateral passage tolerance"},
                {"heading_deg": -3, "lateral_error_m": .02, "ready_for_forward": True,
                 "projected_lateral_bounds_m": [-.01,.04]},
            ],
            "completed_actions": ["double_single_small_strafe_left"],
            "visual_turns": [{"action": "single_micro_right", "before_heading_deg": -21.,
                              "after_heading_deg": -6.}],
        }}}}]}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "左跨中途扫到 Tag 5，转入入口纠偏 · 距码约 24.0 cm · 航向 -6.00° · 横偏 +8.0 cm",
            "Tag 5 入口纠偏 · 航向 -6.00° · 横偏 +8.0 cm · 暂缓直行 · 预计直行横偏超出入口余量",
            "Tag 5 入口纠偏 · 航向 -3.00° · 横偏 +2.0 cm · 预计横偏 -1.0 至 +4.0 cm · 允许下一段直行",
            "Tag 5 入口纠偏 · 已完成 左小跨",
            "Tag 5 视觉转向 · 小右转 · 转前 -21.00° · 转后 -6.00°",
        ))

    def test_tag5_legacy_report_keeps_measured_lateral_offsets(self):
        result = {"ok": True, "corridor_trial": {"delivery": {"entry_lateral_alignment": {
            "observations": [{"map_y_error_m": -.07}, {"map_y_error_m": -.01}],
            "completed_actions": ["single_small_strafe_right"],
        }}}}
        self.assertEqual(task_report.TaskReport_Diagnostics(result),
                         ("Tag 5 进场横向估计 · -7.0 cm → -1.0 cm · 右小跨",))

    def test_tag5_expected_occlusion_reports_continuation(self):
        result = {"ok": False, "corridor_trial": {"delivery": {"entry_lateral_alignment": {
            "observations": [{"ready_for_forward": False, "continued_after_expected_tag_occlusion": True}],
        }}}}
        self.assertEqual(task_report.TaskReport_Diagnostics(result),
                         ("Tag 5 入口纠偏 · 沿已核验路线继续",))

    def test_visual_disagreement_report_displays_both_measured_turns_even_on_success(self):
        result = {"ok": True, "entry_alignment": {"correction_steps": [
            {"heading_disagreement": {"tag_id": 1, "visual_delta_deg": 4.69,
                "magnetic_delta_deg": 9.0, "delta_disagreement_deg": 4.31,
                "selected_source": "trusted_tag_heading"}}, {}
        ]}}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "Tag 1 转角对照 · 视觉 +4.69° · 磁力计 +9.00° · 相差 +4.31° · 以 Tag 航向为准",
        ))
        self.assertEqual(task_report.TaskReport_Message(
            "visual micro stopped at completed macro boundary: visual and magnetic heading changes disagree; "
            "no next route macro, no reverse retry or release; no native immediate stop"),
            "自动任务中断 · 视觉纠偏 · 视觉与磁力计测得的转角不一致")

    def test_magnetic_heading_reference_reports_only_latest_visual_sync_per_tag(self):
        def synchronized(tag, error):
            return {"segment_heading_reference": {"status": "tag_departure_reference_established",
                    "reference": {"tag_departure": {"tag_id": tag, "error_deg": error}}}}

        result = {"ok": True, "events": [
            {"segment_heading_reference": {"status": "relative_reference_established"}},
            synchronized(1, 3.04), synchronized(3, 9.77), synchronized(4, -14.0), synchronized(3, -3.52),
            {"segment_heading_reference": {"status": "reference_unavailable",
                                            "tag_departure": {"tag_id": 4, "error_deg": -6.11}}},
            {"segment_heading_reference": {"status": "reference_unavailable"}},
        ]}
        self.assertEqual(task_report.TaskReport_Diagnostics(result), (
            "Tag 1 · 已用视觉更新磁航向参考 · 视觉航向余差 +3.04°",
            "Tag 3 · 已用视觉更新磁航向参考 · 视觉航向余差 -3.52°",
            "Tag 4 · 磁航向参考待更新 · 视觉航向余差 -6.11°",
        ))

    def test_planned_turn_logs_coarse_margin_and_medium_response_limit_separately(self):
        result = {"ok": False, "events": [
            {"coarse_turn_prediction": {"applied_to_route": True, "use_original_macro": False,
                "allowed_upper_abs_deg": 49, "estimate": {"upper_abs_deg": 50}}},
            {"post_action_heading": {"action": "planned_turn_complete", "correction": "single_medium_right",
                "correction_diagnostic": {"observed_change_deg": 29, "max_step_deg": 28,
                                          "failures": ["observed_step_oversized"]}}},
        ]}
        lines = task_report.TaskReport_Diagnostics(result)
        self.assertIn("实测转角 +29.00° · 单次上限 28°", lines[0])
        self.assertEqual(lines[1], "已切换到小幅转向 · 大转预测上界 50° · 保留航向余量后允许 49°")


if __name__ == "__main__":
    unittest.main()
