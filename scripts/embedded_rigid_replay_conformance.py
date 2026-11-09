"""Native embedded actor/COM twist and paused Fabric Replay regression."""

from __future__ import annotations

import argparse
import json
import tempfile
import traceback
from pathlib import Path

from unirobosim import (
    COMPOSITE_WORLD_SCHEMA_VERSION,
    EmbeddedEntityBinding,
    EmbeddedPrimBinding,
    EntityKind,
    EntityPath,
    EntitySpec,
    EnvironmentSpec,
    PhysicsSpec,
    WorldSpec,
)

from unirobosim_isaaclab import IsaacLabAdapterConfig
from unirobosim_isaaclab.native import IsaacLabNativeRuntime
from unirobosim_isaaclab.native_protocols import NativeRenderRigidBodyState, NativeRenderStateFrame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = {"passed": False, "checks": {}}
    runtime = None
    try:
        with tempfile.TemporaryDirectory(prefix="embedded-native-") as td:
            asset = Path(td, "body.usda")
            asset.write_text("""#usda 1.0
(defaultPrim="Scene"; metersPerUnit=1; upAxis="Z")
def Xform "Scene" {
 def Xform "body" (prepend apiSchemas=["PhysicsRigidBodyAPI","PhysicsMassAPI"]) {
  double3 xformOp:translate=(0,0,1)
  uniform token[] xformOpOrder=["xformOp:translate"]
  float physics:mass=0.1
  point3f physics:centerOfMass=(0.02,0,0)
  float3 physics:diagonalInertia=(0.001,0.001,0.001)
  def Cube "Collider" (prepend apiSchemas=["PhysicsCollisionAPI"]) {
   double size=0.01
  }
 }
}
""")
            container, path = EntityPath("/scene"), EntityPath("/scene/body")
            spec = WorldSpec(
                "embedded-replay-native",
                (
                    EntitySpec(container, EntityKind.COMPOSITE_SCENE, asset_uri=asset.as_uri()),
                    EntitySpec(
                        path,
                        EntityKind.RIGID_BODY,
                        embedded_binding=EmbeddedEntityBinding(
                            container_path=container,
                            root_body_prim_path="body",
                            link_prims=(EmbeddedPrimBinding("body", "body"),),
                        ),
                    ),
                ),
                environments=EnvironmentSpec(2),
                physics=PhysicsSpec(time_step_seconds=1 / 60, gravity_m_s2=(0, 0, 0)),
                schema_version=COMPOSITE_WORLD_SCHEMA_VERSION,
                build_resource_manifest_sha256="0" * 64,
            )
            # Exercise the real native adapter directly; no claimed public asset-manifest admission.
            runtime = IsaacLabNativeRuntime(IsaacLabAdapterConfig(), process_isolated=True)
            world = runtime.build_world(spec)
            before = world.read_rigid_body(path)
            pose = (3.0, 2.0, 1.0)
            quat = (0.0, 0.0, 2**-0.5, 2**-0.5)
            velocity = (0.01, 0.02, 0.03)
            angular = (0.0, 0.0, 2.0)
            tick = world._step_index
            world.apply_render_state(
                NativeRenderStateFrame(
                    (), (NativeRenderRigidBodyState(path, (pose,), (quat,), (velocity,), (angular,), (1,)),), ()
                )
            )
            after = world.read_rigid_body(path)
            result["readback"] = after
            result["raw_com_velocity"] = world._usd_rigid_views[path].get_velocities().cpu().tolist()
            result["checks"]["actor_velocity_roundtrip"] = (
                max(abs(a - b) for a, b in zip(after[2][1], velocity, strict=True)) < 1e-6
            )
            result["checks"]["actor_pose_roundtrip"] = (
                max(abs(a - b) for a, b in zip(after[0][1], pose, strict=True)) < 1e-6
            )
            result["checks"]["com_velocity_correct"] = abs(result["raw_com_velocity"][1][0] + 0.03) < 1e-6
            result["checks"]["unselected_environment_unchanged"] = all(before[i][0] == after[i][0] for i in range(4))
            result["checks"]["paused_no_tick"] = world._step_index == tick
            _, publisher = world._render_transform_publishers["frame",]
            view = publisher._groups[0][0]
            positions, orientations = view.get_world_poses()
            result["fabric_enabled"] = view._use_fabric
            result["fabric_positions"] = positions.torch.cpu().tolist()
            expected = tuple(a + b for a, b in zip(pose, world._origins_cpu[1], strict=True))
            result["checks"]["fabric_world_position"] = (
                max(abs(a - b) for a, b in zip(result["fabric_positions"][1], expected, strict=True)) < 1e-5
            )
            result["checks"]["fabric_path_used"] = bool(view._use_fabric)
            world.set_entity_prim_pose(path, (4.0, 3.0, 2.0), (0.0, 0.0, 0.0, 1.0), 1)
            moved = world.read_rigid_body(path)
            result["checks"]["set_pose_subset"] = moved[0][1] == (4.0, 3.0, 2.0) and moved[0][0] == before[0][0]
            world.reset((1,))
            reset = world.read_rigid_body(path)
            result["checks"]["reset_subset"] = reset == before
            result["passed"] = all(result["checks"].values())
    except BaseException:
        result["error"] = traceback.format_exc()
    Path(args.output).write_text(json.dumps(result, indent=2))
    print("EMBEDDED_RESULT " + json.dumps(result), flush=True)
    if runtime is not None:
        runtime.close()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
