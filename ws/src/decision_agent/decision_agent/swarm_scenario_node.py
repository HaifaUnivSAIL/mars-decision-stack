from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from ros_alate_interfaces.msg import OpCom

from .swarm_geometry import CenterTarget
from .swarm_ros_io import SwarmRosIo
from .swarm_scenario import (
    SwarmScenarioAction,
    compute_body_frame_command,
    compute_initial_center_target,
    desired_drone_pose,
    interpolate_center_target,
    load_swarm_scenario,
    resolve_formation_offsets,
)
from .swarm_world_model import HLC_STATE_LLC_DOWN, HLC_STATE_LOW_BATTERY, MC_STATE_ERROR, SwarmWorldModel


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a formation-based swarm scenario over the existing ROS/Alate contract.")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--scenario-file", required=True, type=Path)
    return parser.parse_args(argv)


class SwarmScenarioNode(Node):
    def __init__(self, manifest_path: Path, scenario_path: Path):
        super().__init__("swarm_scenario")
        self._manifest = json.loads(manifest_path.read_text())
        self._scenario = load_swarm_scenario(scenario_path)
        self._drone_ids = [str(drone["id"]) for drone in self._manifest.get("drones", [])]
        self._formation_offsets = resolve_formation_offsets(self._manifest, self._scenario.formation_offsets)
        self._world_model = SwarmWorldModel(self._drone_ids)
        self._io = SwarmRosIo(self, self._world_model, self._manifest)
        self._profile_enabled = os.getenv("STACK_PROFILE", "0") == "1"

        self._phase = "waiting_ready"
        self._phase_started_sec: Optional[float] = None
        self._ready_observed_sec: Optional[float] = None
        self._scenario_started_sec: Optional[float] = None
        self._desired_center: Optional[CenterTarget] = None
        self._current_action_index = 0
        self._current_action_started_sec: Optional[float] = None
        self._current_action_start_center: Optional[CenterTarget] = None
        self._current_action_metrics: Optional[dict] = None
        self._takeoff_sent = False
        self._takeoff_sent_sec: Optional[float] = None
        self._landing_sent = False
        self._abort_sent = False
        self._finished = False
        self._failed = False
        self._first_airborne_times: dict[str, float] = {}
        self._error_baseline: dict[str, str] = {drone_id: "" for drone_id in self._drone_ids}

        period_sec = 1.0 / max(self._scenario.control_frequency_hz, 0.1)
        self.create_timer(period_sec, self._tick)
        self.get_logger().info(
            f"swarm_scenario started scenario={self._scenario.scenario_name} drones={len(self._drone_ids)} "
            f"auto_takeoff={self._scenario.auto_takeoff}"
        )

    def is_finished(self) -> bool:
        return self._finished

    def failed(self) -> bool:
        return self._failed

    def _tick(self) -> None:
        if self._finished:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._phase_started_sec is None:
            self._enter_phase("waiting_ready", now_sec)

        if self._phase == "waiting_ready":
            self._handle_waiting_ready(now_sec)
        elif self._phase == "waiting_airborne":
            self._handle_waiting_airborne(now_sec)
        elif self._phase == "running_actions":
            self._handle_running_actions(now_sec)
        elif self._phase == "landing":
            self._handle_landing(now_sec)
        elif self._phase == "aborting":
            self._handle_aborting(now_sec)

    def _handle_waiting_ready(self, now_sec: float) -> None:
        if not self._world_model.all_runtime_ready(
            self._drone_ids,
            now_sec,
            self._scenario.telemetry_timeout_sec,
            self._scenario.require_ready_state,
        ):
            self._ready_observed_sec = None
            return

        if not self._world_model.has_local_origin():
            active_drone_id = str(self._manifest.get("active_drone_id") or self._drone_ids[0])
            if not self._world_model.set_local_origin_from_drone(active_drone_id):
                return
            self._desired_center = self._current_center_target()
            if self._desired_center is None:
                return

        if self._ready_observed_sec is None:
            self._ready_observed_sec = now_sec
            self.get_logger().info("All drones ready; holding briefly before swarm start")
            return
        if (now_sec - self._ready_observed_sec) < self._scenario.ready_hold_sec:
            return

        if self._scenario.auto_takeoff:
            if not self._takeoff_sent:
                self._io.publish_operator_command_all(self._drone_ids, OpCom.OP_COMMAND_TAKEOFF)
                self._takeoff_sent = True
                self._takeoff_sent_sec = now_sec
                self._snapshot_error_baseline()
                self._profile("group_takeoff_published", drone_ids=self._drone_ids)
            self._enter_phase("waiting_airborne", now_sec)
            return

        self._scenario_started_sec = now_sec
        self._enter_phase("running_actions", now_sec)

    def _handle_waiting_airborne(self, now_sec: float) -> None:
        issue = self._first_mission_issue(now_sec)
        if issue is not None:
            self._enter_abort(now_sec, issue[0], issue[1])
            return

        for drone_id in self._drone_ids:
            if drone_id in self._first_airborne_times:
                continue
            if self._world_model.drone_state(drone_id).runtime.is_airborne(
                now_sec,
                self._scenario.telemetry_timeout_sec,
                self._scenario.takeoff_altitude_m,
            ):
                self._first_airborne_times[drone_id] = now_sec

        if self._world_model.all_airborne(
            self._drone_ids,
            now_sec,
            self._scenario.telemetry_timeout_sec,
            self._scenario.takeoff_altitude_m,
        ):
            self._scenario_started_sec = now_sec
            spread_sec = 0.0
            if self._first_airborne_times:
                spread_sec = max(self._first_airborne_times.values()) - min(self._first_airborne_times.values())
            self._profile("group_takeoff_completed", takeoff_spread_sec=spread_sec, airborne_times=self._first_airborne_times)
            self._enter_phase("running_actions", now_sec)
            return

        if self._takeoff_sent_sec is not None and (now_sec - self._takeoff_sent_sec) > self._scenario.takeoff_timeout_sec:
            self._enter_abort(now_sec, "group_takeoff_timeout", "")

    def _handle_running_actions(self, now_sec: float) -> None:
        issue = self._first_mission_issue(now_sec)
        if issue is not None:
            self._enter_abort(now_sec, issue[0], issue[1])
            return

        if self._current_action_index >= len(self._scenario.actions):
            if self._scenario.auto_land_after_actions:
                self._send_land_all(now_sec, "auto_land_after_actions")
                self._enter_phase("landing", now_sec)
            else:
                self._finish(now_sec)
            return

        action = self._scenario.actions[self._current_action_index]
        if action.kind == "land":
            self._send_land_all(now_sec, "explicit_land_action")
            self._complete_action(now_sec, action, CenterTarget(0.0, 0.0, 0.0, 0.0), CenterTarget(0.0, 0.0, 0.0, 0.0))
            self._enter_phase("landing", now_sec)
            return

        if self._current_action_started_sec is None:
            self._current_action_started_sec = now_sec
            self._current_action_start_center = self._desired_center
            self._current_action_metrics = self._new_action_metrics()
            self._profile(
                "swarm_action_started",
                action_index=self._current_action_index + 1,
                action_kind=action.kind,
                duration_sec=action.duration_sec,
                label=action.label,
            )

        assert self._current_action_start_center is not None
        assert self._current_action_metrics is not None
        elapsed_sec = now_sec - self._current_action_started_sec
        progress = 1.0 if action.duration_sec <= 0.0 else min(elapsed_sec / action.duration_sec, 1.0)
        target_center = interpolate_center_target(self._current_action_start_center, action, progress)
        self._apply_controller(target_center)
        self._update_action_metrics(target_center)

        if elapsed_sec >= action.duration_sec:
            self._complete_action(now_sec, action, self._current_action_start_center, target_center)
            self._desired_center = target_center
            self._current_action_index += 1
            self._current_action_started_sec = None
            self._current_action_start_center = None
            self._current_action_metrics = None

    def _handle_landing(self, now_sec: float) -> None:
        self._io.publish_zero_all(self._drone_ids)
        if self._world_model.all_landed(
            self._drone_ids,
            now_sec,
            self._scenario.telemetry_timeout_sec,
            self._scenario.landed_altitude_m,
        ):
            self._finish(now_sec)
            return
        if (now_sec - (self._phase_started_sec or now_sec)) > self._scenario.landing_timeout_sec:
            self._fail(now_sec, "Timed out waiting for group landing")

    def _handle_aborting(self, now_sec: float) -> None:
        self._io.publish_zero_all(self._drone_ids)
        if self._world_model.all_landed(
            self._drone_ids,
            now_sec,
            self._scenario.telemetry_timeout_sec,
            self._scenario.landed_altitude_m,
        ):
            self._finish(now_sec)
            return
        if (now_sec - (self._phase_started_sec or now_sec)) > self._scenario.landing_timeout_sec:
            self._fail(now_sec, "Group abort landing timeout elapsed")

    def _current_center_target(self) -> Optional[CenterTarget]:
        local_poses = {}
        for drone_id in self._drone_ids:
            pose = self._world_model.local_pose(drone_id)
            if pose is None:
                return None
            local_poses[drone_id] = pose
        return compute_initial_center_target(local_poses, self._formation_offsets)

    def _apply_controller(self, target_center: CenterTarget) -> None:
        for drone_id in self._drone_ids:
            actual_pose = self._world_model.local_pose(drone_id)
            if actual_pose is None:
                continue
            desired_pose = desired_drone_pose(target_center, self._formation_offsets[drone_id])
            command, _error_metrics = compute_body_frame_command(actual_pose, desired_pose, self._scenario.controller)
            self._io.publish_twist(
                drone_id,
                linear_x=command["linear_x"],
                linear_y=command["linear_y"],
                linear_z=command["linear_z"],
                angular_z=command["angular_z"],
            )

    def _new_action_metrics(self) -> dict:
        return {
            "samples": 0,
            "per_drone": {
                drone_id: {
                    "max_horizontal_error_m": 0.0,
                    "max_vertical_error_m": 0.0,
                    "max_yaw_error_deg": 0.0,
                }
                for drone_id in self._drone_ids
            },
        }

    def _update_action_metrics(self, target_center: CenterTarget) -> None:
        if self._current_action_metrics is None:
            return
        self._current_action_metrics["samples"] += 1
        for drone_id in self._drone_ids:
            actual_pose = self._world_model.local_pose(drone_id)
            if actual_pose is None:
                continue
            desired_pose = desired_drone_pose(target_center, self._formation_offsets[drone_id])
            _command, error_metrics = compute_body_frame_command(actual_pose, desired_pose, self._scenario.controller)
            metrics = self._current_action_metrics["per_drone"][drone_id]
            metrics["max_horizontal_error_m"] = max(metrics["max_horizontal_error_m"], error_metrics["horizontal_error_m"])
            metrics["max_vertical_error_m"] = max(metrics["max_vertical_error_m"], error_metrics["vertical_error_m"])
            metrics["max_yaw_error_deg"] = max(metrics["max_yaw_error_deg"], error_metrics["yaw_error_deg"])

    def _complete_action(
        self,
        now_sec: float,
        action: SwarmScenarioAction,
        start_center: CenterTarget,
        end_center: CenterTarget,
    ) -> None:
        metrics = self._current_action_metrics or self._new_action_metrics()
        aggregate_horizontal = max(
            item["max_horizontal_error_m"] for item in metrics["per_drone"].values()
        ) if metrics["per_drone"] else 0.0
        aggregate_vertical = max(
            item["max_vertical_error_m"] for item in metrics["per_drone"].values()
        ) if metrics["per_drone"] else 0.0
        aggregate_yaw = max(
            item["max_yaw_error_deg"] for item in metrics["per_drone"].values()
        ) if metrics["per_drone"] else 0.0
        self._profile(
            "swarm_action_completed",
            action_index=self._current_action_index + 1,
            action_kind=action.kind,
            label=action.label,
            duration_sec=action.duration_sec,
            start_center=start_center.__dict__,
            end_center=end_center.__dict__,
            sample_count=metrics["samples"],
            aggregate_max_horizontal_error_m=aggregate_horizontal,
            aggregate_max_vertical_error_m=aggregate_vertical,
            aggregate_max_yaw_error_deg=aggregate_yaw,
            per_drone=metrics["per_drone"],
        )
        self._io.publish_zero_all(self._drone_ids)

    def _send_land_all(self, now_sec: float, reason: str) -> None:
        if self._landing_sent:
            return
        self._landing_sent = True
        self._io.publish_operator_command_all(self._drone_ids, OpCom.OP_COMMAND_LAND)
        self._profile("land_all_published", reason=reason)
        self._io.publish_zero_all(self._drone_ids)

    def _enter_abort(self, now_sec: float, reason: str, drone_id: str) -> None:
        if self._abort_sent:
            return
        self._abort_sent = True
        self._failed = True
        self._io.publish_zero_all(self._drone_ids)
        self._io.publish_operator_command_all(self._drone_ids, OpCom.OP_COMMAND_LAND)
        self._profile("group_abort_triggered", reason=reason, drone_id=drone_id)
        self._enter_phase("aborting", now_sec)

    def _first_mission_issue(self, now_sec: float) -> tuple[str, str] | None:
        for drone_id in self._drone_ids:
            state = self._world_model.drone_state(drone_id)
            if not state.runtime.telemetry_is_fresh(now_sec, self._scenario.telemetry_timeout_sec):
                return "telemetry_stale", drone_id
            if state.runtime.last_error and state.runtime.last_error != self._error_baseline.get(drone_id, ""):
                return "platform_error", drone_id
            if state.runtime.mc_state == MC_STATE_ERROR:
                return "mission_control_error", drone_id
            if state.runtime.hlc_state in {HLC_STATE_LOW_BATTERY, HLC_STATE_LLC_DOWN}:
                return "high_level_control_error", drone_id
        return None

    def _snapshot_error_baseline(self) -> None:
        self._error_baseline = {
            drone_id: self._world_model.drone_state(drone_id).runtime.last_error
            for drone_id in self._drone_ids
        }

    def _enter_phase(self, phase: str, now_sec: float) -> None:
        self._phase = phase
        self._phase_started_sec = now_sec
        self.get_logger().info(f"Swarm phase -> {phase}")
        self._profile("scenario_phase_entered", phase=phase)

    def _finish(self, now_sec: float) -> None:
        self._io.publish_zero_all(self._drone_ids)
        total_sec = 0.0 if self._scenario_started_sec is None else now_sec - self._scenario_started_sec
        self._finished = True
        self.get_logger().info(f"Swarm scenario finished after {total_sec:.2f}s failed={self._failed}")
        self._profile("scenario_finished", total_duration_sec=total_sec, failed=self._failed)

    def _fail(self, now_sec: float, reason: str) -> None:
        self.get_logger().error(reason)
        self._failed = True
        self._finish(now_sec)

    def _profile(self, event_type: str, **payload) -> None:
        if not self._profile_enabled:
            return
        payload.update(
            {
                "component": "swarm_scenario",
                "event_type": event_type,
                "wall_time_epoch_sec": time.time(),
                "ros_time_sec": self.get_clock().now().nanoseconds / 1e9,
                "scenario_name": self._scenario.scenario_name,
            }
        )
        self.get_logger().info(f"PROFILE:{json.dumps(payload, separators=(',', ':'))}")


def main(args=None) -> int:
    rclpy.init(args=args)
    cli_args = parse_args(rclpy.utilities.remove_ros_args(args=args)[1:])
    node = SwarmScenarioNode(cli_args.manifest, cli_args.scenario_file)
    exit_code = 0
    try:
        while rclpy.ok() and not node.is_finished():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node.failed():
            exit_code = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
