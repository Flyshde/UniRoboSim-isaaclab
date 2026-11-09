"""Native, finite D6 connectors; not kinematic or rigid attachment ownership."""

from __future__ import annotations

import hashlib
import logging
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from unirobosim import EntityPath, Pose

_LOG = logging.getLogger(__name__)

if TYPE_CHECKING:
    from unirobosim import ConnectionProperties


@dataclass(frozen=True)
class Connection:
    parent: EntityPath
    child: EntityPath
    environment: int
    rest: Pose
    properties: ConnectionProperties
    joint_path: str


def error(relative: Pose, rest: Pose) -> tuple[float, float]:
    distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(relative.position, rest.position, strict=True)))
    dot = abs(sum(a * b for a, b in zip(relative.orientation_xyzw, rest.orientation_xyzw, strict=True)))
    return distance, 2 * math.acos(min(1.0, max(0.0, dot)))


def drive_parameters(properties: ConnectionProperties, angular: bool) -> tuple[float, float, float]:
    # USD angular drive targets use degrees; force limits remain N m. Per-axis
    # caps /sqrt(3) bound the full three-dimensional force/torque vector.
    if angular:
        conversion = math.pi / 180.0
        return (
            properties.angular_stiffness_nm_rad * conversion,
            properties.angular_damping_nms_rad * conversion,
            properties.max_torque_nm / math.sqrt(3.0),
        )
    return properties.stiffness_n_m, properties.damping_ns_m, properties.max_force_n / math.sqrt(3.0)


def create(
    native: Any,
    key: tuple[int, str],
    parent: EntityPath,
    child: EntityPath,
    rest: Pose,
    properties: ConnectionProperties,
    relative: Pose,
) -> str | None:
    environment, connection_id = key
    if key in native._runtime_connections:
        return "connection_exists"
    if parent == child or any(
        c.child == child and c.environment == environment for c in native._runtime_connections.values()
    ):
        return "connection_child_unavailable"
    distance, angle = error(relative, rest)
    if distance > properties.admission_position_m or angle > properties.admission_angle_rad:
        return "connection_not_seated"
    m = native._m
    stage = m.sim_utils.get_current_stage()
    parent_body = native._attachment_body_path(parent, None, environment)
    child_body = native._attachment_body_path(child, None, environment)
    if parent_body == child_body:
        return "connection_same_body"
    root = f"/World/env_{environment}/unirobosim_runtime_connections"
    path = root + "/spring_" + hashlib.sha256(connection_id.encode()).hexdigest()[:24]
    if stage.GetPrimAtPath(path).IsValid():
        return "connection_path_occupied"
    stage.DefinePrim(root, "Scope")
    joint = m.UsdPhysics.Joint.Define(stage, path)
    try:
        joint.CreateBody0Rel().SetTargets([m.Sdf.Path(parent_body)])
        joint.CreateBody1Rel().SetTargets([m.Sdf.Path(child_body)])
        joint.CreateLocalPos0Attr().Set(m.Gf.Vec3f(*rest.position))
        q = rest.orientation_xyzw
        joint.CreateLocalRot0Attr().Set(m.Gf.Quatf(q[3], m.Gf.Vec3f(*q[:3])))
        joint.CreateLocalPos1Attr().Set(m.Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(m.Gf.Quatf(1.0, m.Gf.Vec3f(0.0, 0.0, 0.0)))
        joint.CreateCollisionEnabledAttr().Set(True)
        joint.CreateExcludeFromArticulationAttr().Set(True)
        for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
            stiffness, damping, cap = drive_parameters(properties, axis.startswith("rot"))
            drive = m.UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(stiffness)
            drive.CreateDampingAttr().Set(damping)
            drive.CreateMaxForceAttr().Set(cap)
            drive.CreateTargetPositionAttr().Set(0.0)
            drive.CreateTargetVelocityAttr().Set(0.0)
        joint.CreateJointEnabledAttr().Set(True)
        native._sim.forward()
    except BaseException:
        stage.RemovePrim(path)
        raise
    native._runtime_connections[key] = Connection(parent, child, environment, rest, properties, path)
    _LOG.info("compliant connector %s admitted: distance_m=%.9g angle_rad=%.9g", key, distance, angle)
    return None


def clear(native: Any, environments: tuple[int, ...] | None = None) -> None:
    if not getattr(native, "_runtime_connections", None):
        return
    stage = native._m.sim_utils.get_current_stage()
    for key, connection in tuple(native._runtime_connections.items()):
        if environments is None or connection.environment in environments:
            stage.RemovePrim(connection.joint_path)
            del native._runtime_connections[key]


def update(native: Any, relative_pose: Any) -> None:
    """Check the break envelope at every native physics step, never wall time."""
    stage = native._m.sim_utils.get_current_stage()
    for key, connection in tuple(native._runtime_connections.items()):
        parent = native._attachment_endpoint_pose(connection.parent, None, connection.environment)
        child = native._attachment_endpoint_pose(connection.child, None, connection.environment)
        distance, angle = error(relative_pose(parent, child), connection.rest)
        if distance > connection.properties.break_distance_m or angle > connection.properties.break_angle_rad:
            _LOG.warning("compliant connector %s broke: distance_m=%.9g angle_rad=%.9g", key, distance, angle)
            stage.RemovePrim(connection.joint_path)
            del native._runtime_connections[key]


def update_declared(native: Any, relative_pose: Any) -> None:
    """Seat-gated passive activation works equally with Mission or a model.

    A broken connection is not retried. Prescribed kinematic Replay endpoints
    need no forces and are left untouched; a partly kinematic declaration is
    still supported (for example a connector in a stationary appliance).
    """
    for declaration in getattr(native._spec, "compliant_connections", ()):
        if native._kinematic_rigids.get(declaration.parent_path, False) and native._kinematic_rigids.get(
            declaration.child_path, False
        ):
            continue
        for environment in range(native._spec.environments.count):
            key = (environment, declaration.connection_id)
            if key in native._admitted_connections:
                continue
            if not all(native._entity_prim_is_physical_root(path, environment)
                       for path in (declaration.parent_path, declaration.child_path)):
                raise ValueError("compliant connections require physical asset-root endpoints")
            parent = native._attachment_endpoint_pose(declaration.parent_path, None, environment)
            child = native._attachment_endpoint_pose(declaration.child_path, None, environment)
            relative = relative_pose(parent, child)
            distance, angle = error(relative, declaration.rest_pose)
            props = declaration.properties
            if distance > props.admission_position_m or angle > props.admission_angle_rad:
                continue
            native._pin_physics_activation(declaration.parent_path)
            native._pin_physics_activation(declaration.child_path)
            rejection = create(
                native, key, declaration.parent_path, declaration.child_path, declaration.rest_pose, props, relative
            )
            if rejection is not None:
                raise ValueError(f"declared connector {declaration.connection_id}: {rejection}")
            native._admitted_connections.add(key)
