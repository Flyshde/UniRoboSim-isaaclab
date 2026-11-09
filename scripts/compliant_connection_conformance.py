"""Native finite spring smoke; temporary assets, no SDK changes, physics 60 Hz."""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import traceback
from pathlib import Path

from unirobosim import (
    CompliantConnectionSpec,
    ConnectionProperties,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PhysicsSpec,
    Pose,
    SceneCommand,
    SceneCommandKind,
    WorldSpec,
)

from unirobosim_isaaclab import IsaacLabAdapterConfig, IsaacLabProvider
from unirobosim_isaaclab.native import IsaacLabNativeRuntime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = {"passed": False, "checks": {}, "samples": {}, "physics_hz": 60, "command_hz": 30}
    try:
        with tempfile.TemporaryDirectory(prefix="spring-conformance-") as td:
            for name, kinematic in [("parent", True), ("child", False)]:
                Path(td, name + ".usda").write_text(
                    """#usda 1.0
(defaultPrim="Body"; metersPerUnit=1; upAxis="Z")
def Xform "Body" (prepend apiSchemas=["PhysicsRigidBodyAPI","PhysicsMassAPI"]) {
 bool physics:kinematicEnabled="""
                    + str(kinematic).lower()
                    + """
 float physics:mass=0.1
 float3 physics:diagonalInertia=(0.001,0.001,0.001)
 def Cube "Collider" (prepend apiSchemas=["PhysicsCollisionAPI"]) {
 double size=0.01
 }
}
"""
                )
            parent, child = EntityPath("/parent"), EntityPath("/child")
            props = ConnectionProperties(300.0, 1.0, 1.0, 0.002, 3.0, 0.01, 0.008, 0.35, 0.0015, 0.02)
            spec = WorldSpec(
                "spring-native",
                (
                    EntitySpec(
                        parent, EntityKind.RIGID_BODY, asset_uri=str(Path(td, "parent.usda")), pose=Pose((0, 0, 1))
                    ),
                    EntitySpec(
                        child, EntityKind.RIGID_BODY, asset_uri=str(Path(td, "child.usda")), pose=Pose((0.02, 0, 1.1))
                    ),
                ),
                environments=EnvironmentSpec(2),
                physics=PhysicsSpec(time_step_seconds=1 / 60, gravity_m_s2=(0, 0, -1)),
                compliant_connections=(CompliantConnectionSpec("socket", parent, child, Pose((0, 0, 0.1)), props),),
            )
            provider = IsaacLabProvider(
                IsaacLabAdapterConfig(), runtime_factory=lambda c: IsaacLabNativeRuntime(c, process_isolated=True)
            )
            with provider.open() as session:
                world = session.build(spec)
                native = world._native

                def sample(label):
                    p, q, v, w = native.read_rigid_body(child)
                    row = {
                        "position": p[0],
                        "linear_velocity": v[0],
                        "angular_velocity": w[0],
                        "active": int((0, "socket") in native._runtime_connections),
                        "admitted": int((0, "socket") in native._admitted_connections),
                    }
                    result["samples"][label] = row
                    return row

                sequence = 0

                def place(position, orientation=(0, 0, 0, 1), environment=0):
                    nonlocal sequence
                    sequence += 1
                    world.apply_scene_command(
                        SceneCommand(
                            str(sequence),
                            "conformance",
                            "conformance",
                            world.generation,
                            SceneCommandKind.SET_POSE,
                            child,
                            environment_index=environment,
                            target_pose=Pose(position, orientation),
                        )
                    )

                world.step(2)
                before = sample("unseated")
                result["checks"]["no_attraction_unseated"] = (
                    before["active"] == 0 and abs(before["position"][0] - 0.02) < 1e-6
                )
                place((0, 0, 1.1), (math.sin(0.03), 0, 0, math.cos(0.03)))
                world.step(2)
                result["checks"]["angle_only_not_seated"] = sample("angle_unseated")["active"] == 0
                place((0, 0, 1.1))
                world.step(2)
                admitted = sample("admitted")
                result["checks"]["seat_admission"] = admitted["active"] == 1
                for _ in range(60):
                    world.step(2)
                retained = sample("retained_2s")
                result["checks"]["gravity_retention"] = (
                    retained["active"] == 1 and abs(retained["position"][2] - (1.1 - 0.1 / 300)) < 0.001
                )
                place((0.007, 0, 1.1))
                world.step(2)
                force = sample("finite_force")
                measured = abs(0.1 * force["linear_velocity"][0] / (2 / 60))
                result["mean_force_x_n"] = measured
                result["checks"]["finite_nonzero_force"] = 0.01 < measured <= props.max_force_n * 1.01
                place((0, 0, 1.1), (math.sin(0.15), 0, 0, math.cos(0.15)))
                world.step(2)
                torque = sample("angular_saturation")
                measured_torque = abs(0.001 * torque["angular_velocity"][0] / (2 / 60))
                result["mean_torque_x_nm"] = measured_torque
                result["checks"]["angular_torque_cap"] = (
                    abs(measured_torque - props.max_torque_nm / math.sqrt(3)) < 0.0001
                )
                place((0.02, 0.02, 1.12))
                world.step(2)
                broken = sample("broken")
                result["checks"]["break"] = broken["active"] == 0
                # One saturated physics impulse, then one free step after break.
                # Remove the two-step gravity impulse to isolate the spring.
                impulse_velocity = list(broken["linear_velocity"])
                impulse_velocity[2] += 2 / 60
                cap_force = math.sqrt(sum((0.1 * value * 60) ** 2 for value in impulse_velocity))
                result["single_step_resultant_force_n"] = cap_force
                result["checks"]["saturated_resultant_force_cap"] = abs(cap_force - props.max_force_n) < 0.01
                place((0, 0, 1.1))
                world.step(2)
                result["checks"]["no_automatic_reconnection"] = sample("reseated")["active"] == 0
                world.reset((0,))
                reset = sample("reset")
                result["checks"]["reset_clears_history"] = reset["active"] == 0 and reset["admitted"] == 0
                place((0, 0, 1.1))
                world.step(2)
                result["checks"]["next_episode_admits"] = sample("next_episode")["active"] == 1
                place((0, 0, 1.1), (math.sin(0.25), 0, 0, math.cos(0.25)))
                world.step(2)
                result["checks"]["angular_only_break"] = sample("angular_broken")["active"] == 0
                place((0, 0, 1.1), environment=1)
                world.step(2)
                second_path = native._runtime_connections[1, "socket"].joint_path
                second_state = native.read_rigid_body(child)[0][1]
                world.reset((0,))
                result["checks"]["subset_reset_preserves_other_connector"] = (
                    (1, "socket") in native._runtime_connections
                    and native._runtime_connections[1, "socket"].joint_path == second_path
                    and native.read_rigid_body(child)[0][1] == second_state
                    and (1, "socket") in native._admitted_connections
                    and (0, "socket") not in native._admitted_connections
                )
                try:
                    native.capture_checkpoint()
                except ValueError as exc:
                    result["checks"]["checkpoint_rejected"] = "compliant connection" in str(exc)
                else:
                    result["checks"]["checkpoint_rejected"] = False
                result["passed"] = all(result["checks"].values())
                Path(args.output).write_text(json.dumps(result, indent=2))
    except BaseException:
        result["error"] = traceback.format_exc()
    Path(args.output).write_text(json.dumps(result, indent=2))
    print("CONNECTION_RESULT " + json.dumps(result), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
