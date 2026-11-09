import math
import subprocess
import sys
from types import SimpleNamespace

import pytest
from unirobosim import CompliantConnectionSpec, ConnectionProperties, EntityPath, Pose

from unirobosim_isaaclab.compliant_connection import create, drive_parameters, error, update, update_declared


def properties():
    return ConnectionProperties(300.0, 1.0, 1.0, 0.002, 3.0, 0.01, 0.008, 0.35, 0.0015, 0.02)


def test_rotational_drive_units_and_vector_force_caps():
    stiffness, damping, cap = drive_parameters(properties(), True)
    assert stiffness * 180 / math.pi == pytest.approx(1.0)
    assert damping * 180 / math.pi == pytest.approx(0.002)
    assert math.sqrt(3 * cap**2) == pytest.approx(0.01)
    stiffness, damping, cap = drive_parameters(properties(), False)
    assert (stiffness, damping) == (300.0, 1.0)
    assert math.sqrt(3 * cap**2) == pytest.approx(3.0)


@pytest.mark.parametrize(
    "position,quaternion",
    [((0.002, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), ((0.0, 0.0, 0.0), (math.sin(0.02), 0.0, 0.0, math.cos(0.02)))],
)
def test_unseated_connection_is_rejected_before_touching_stage(position, quaternion):
    native = SimpleNamespace(_runtime_connections={})
    assert (
        create(
            native,
            (0, "socket"),
            EntityPath("/phone"),
            EntityPath("/plug"),
            Pose((0.0, 0.0, 0.0)),
            properties(),
            Pose(position, quaternion),
        )
        == "connection_not_seated"
    )
    assert native._runtime_connections == {}


def test_joint_preserves_collision_and_uses_only_finite_implicit_drives():
    code = "from tests.test_compliant_connection import _check_native_joint; _check_native_joint()"
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def _check_native_joint():
    from pxr import Gf, Sdf, Usd, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/phone", "Xform")
    stage.DefinePrim("/plug", "Xform")
    native = SimpleNamespace(
        _runtime_connections={},
        _attachment_body_path=lambda path, *_: path.value,
        _sim=SimpleNamespace(forward=lambda: None),
        _m=SimpleNamespace(
            Gf=Gf, Sdf=Sdf, UsdPhysics=UsdPhysics, sim_utils=SimpleNamespace(get_current_stage=lambda: stage)
        ),
    )
    rest = Pose((0.0, -0.082, 0.0))
    create(native, (0, "socket"), EntityPath("/phone"), EntityPath("/plug"), rest, properties(), rest)
    connection = native._runtime_connections[0, "socket"]
    joint = UsdPhysics.Joint.Get(stage, connection.joint_path)
    assert not joint.GetPrim().IsA(UsdPhysics.FixedJoint)
    assert joint.GetCollisionEnabledAttr().Get() is True
    assert joint.GetExcludeFromArticulationAttr().Get() is True
    assert list(joint.GetLocalPos0Attr().Get()) == pytest.approx(rest.position)
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        drive = UsdPhysics.DriveAPI(joint.GetPrim(), axis)
        assert drive.GetTypeAttr().Get() == "force"
        assert 0 < drive.GetMaxForceAttr().Get() < 3
    native._attachment_endpoint_pose = lambda path, *_: Pose((0.0, 0.0, 0.0))
    update(native, lambda *_: Pose((0.009, -0.082, 0.0)))
    assert native._runtime_connections == {}
    assert not stage.GetPrimAtPath(connection.joint_path).IsValid()


def test_quaternion_sign_is_not_a_break_or_admission_error():
    assert error(Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, -1.0)), Pose((0.0, 0.0, 0.0))) == (0.0, 0.0)


def test_passive_admission_without_mission_never_attracts_unseated_or_retries_break(monkeypatch):
    import unirobosim_isaaclab.compliant_connection as module

    c = CompliantConnectionSpec(
        "socket", EntityPath("/phone"), EntityPath("/plug"), Pose((0.0, -0.082, 0.0)), properties()
    )
    native = SimpleNamespace(
        _spec=SimpleNamespace(compliant_connections=(c,), environments=SimpleNamespace(count=1)),
        _kinematic_rigids={},
        _admitted_connections=set(),
        _attachment_endpoint_pose=lambda *_: Pose(),
        _pin_physics_activation=lambda *_: None,
        _entity_prim_is_physical_root=lambda *_: True,
    )
    calls = []
    monkeypatch.setattr(module, "create", lambda *args: calls.append(args))
    update_declared(native, lambda *_: Pose((0.0, -0.09, 0.0)))
    assert not calls and not native._admitted_connections
    update_declared(native, lambda *_: c.rest_pose)
    assert len(calls) == 1 and native._admitted_connections == {(0, "socket")}
    # A broken D6 may disappear, but admission history persists for this episode.
    update_declared(native, lambda *_: c.rest_pose)
    assert len(calls) == 1
    # Reset clears history and permits the next episode, not a physical retry.
    native._admitted_connections.clear()
    update_declared(native, lambda *_: c.rest_pose)
    assert len(calls) == 2


def test_prescribed_kinematic_replay_has_no_connector_forces(monkeypatch):
    import unirobosim_isaaclab.compliant_connection as module

    c = CompliantConnectionSpec("socket", EntityPath("/phone"), EntityPath("/plug"), Pose(), properties())
    native = SimpleNamespace(
        _spec=SimpleNamespace(compliant_connections=(c,)), _kinematic_rigids={c.parent_path: True, c.child_path: True}
    )
    monkeypatch.setattr(module, "create", lambda *_: pytest.fail("Replay must not create physics drives"))
    update_declared(native, lambda *_: pytest.fail("Replay must not sample physics for connectors"))


def test_wrapped_asset_root_is_rejected_before_using_body_frame_as_asset_frame():
    c = CompliantConnectionSpec("socket", EntityPath("/phone"), EntityPath("/plug"), Pose(), properties())
    native = SimpleNamespace(_spec=SimpleNamespace(compliant_connections=(c,),
                                                  environments=SimpleNamespace(count=1)),
                             _kinematic_rigids={}, _admitted_connections=set(),
                             _entity_prim_is_physical_root=lambda *_: False)
    with pytest.raises(ValueError, match="physical asset-root endpoints"):
        update_declared(native, lambda *_: pytest.fail("No frame substitution is permitted"))
