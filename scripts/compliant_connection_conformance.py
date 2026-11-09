"""Native finite spring smoke; temporary assets, no SDK changes, physics 60 Hz."""
from __future__ import annotations
import argparse
import json
import tempfile
import traceback
from pathlib import Path
from unirobosim import (CompliantConnectionSpec, ConnectionProperties, EntityKind, EntityPath,
                       EntitySpec, PhysicsSpec, Pose, SceneCommand, SceneCommandKind, WorldSpec)
from unirobosim_isaaclab import IsaacLabAdapterConfig, IsaacLabProvider
from unirobosim_isaaclab.native import IsaacLabNativeRuntime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = {'passed': False, 'checks': {}, 'samples': {}, 'physics_hz': 60, 'command_hz': 30}
    try:
        with tempfile.TemporaryDirectory(prefix='spring-conformance-') as td:
            for name, kinematic in [('parent', True), ('child', False)]:
                Path(td, name+'.usda').write_text('''#usda 1.0
(defaultPrim="Body"; metersPerUnit=1; upAxis="Z")
def Xform "Body" (prepend apiSchemas=["PhysicsRigidBodyAPI","PhysicsMassAPI"]) {
 bool physics:kinematicEnabled='''+str(kinematic).lower()+'''
 float physics:mass=0.1
 def Cube "Collider" (prepend apiSchemas=["PhysicsCollisionAPI"]) {
 double size=0.01
 }
}
''')
            parent, child = EntityPath('/parent'), EntityPath('/child')
            props = ConnectionProperties(300., 1., 1., .002, 3., .01, .008, .35, .0015, .02)
            spec = WorldSpec('spring-native', (
                EntitySpec(parent, EntityKind.RIGID_BODY, asset_uri=str(Path(td,'parent.usda')), pose=Pose((0,0,1))),
                EntitySpec(child, EntityKind.RIGID_BODY, asset_uri=str(Path(td,'child.usda')), pose=Pose((.02,0,1.1))),
            ), physics=PhysicsSpec(time_step_seconds=1/60, gravity_m_s2=(0,0,-1)),
                compliant_connections=(CompliantConnectionSpec('socket', parent, child, Pose((0,0,.1)), props),))
            provider = IsaacLabProvider(IsaacLabAdapterConfig(),
                runtime_factory=lambda c: IsaacLabNativeRuntime(c, process_isolated=True))
            with provider.open() as session:
                world = session.build(spec)
                native = world._native
                def sample(label):
                    p,q,v,w = native.read_rigid_body(child)
                    row = {'position':p[0], 'linear_velocity':v[0], 'angular_velocity':w[0],
                           'active':len(native._runtime_connections), 'admitted':len(native._admitted_connections)}
                    result['samples'][label] = row
                    return row
                sequence = 0
                def place(position):
                    nonlocal sequence
                    sequence += 1
                    world.apply_scene_command(SceneCommand(str(sequence), 'conformance','conformance',world.generation,
                        SceneCommandKind.SET_POSE, child, environment_index=0, target_pose=Pose(position)))
                world.step(2)
                before = sample('unseated')
                result['checks']['no_attraction_unseated'] = before['active']==0 and abs(before['position'][0]-.02)<1e-6
                place((0,0,1.1))
                world.step(2)
                admitted = sample('admitted')
                result['checks']['seat_admission'] = admitted['active']==1
                for _ in range(60):
                    world.step(2)
                retained = sample('retained_2s')
                result['checks']['gravity_retention'] = retained['active']==1 and abs(retained['position'][2]-(1.1-.1/300))<.001
                place((.007,0,1.1))
                world.step(2)
                force = sample('finite_force')
                measured = abs(.1*force['linear_velocity'][0]/(2/60))
                result['mean_force_x_n'] = measured
                result['checks']['finite_nonzero_force'] = .01 < measured <= props.max_force_n*1.01
                place((.02,0,1.1))
                world.step(2)
                result['checks']['break'] = sample('broken')['active']==0
                place((0,0,1.1))
                world.step(2)
                result['checks']['no_automatic_reconnection'] = sample('reseated')['active']==0
                world.reset((0,))
                reset = sample('reset')
                result['checks']['reset_clears_history'] = reset['active']==0 and reset['admitted']==0
                place((0,0,1.1))
                world.step(2)
                result['checks']['next_episode_admits'] = sample('next_episode')['active']==1
                result['passed'] = all(result['checks'].values())
                Path(args.output).write_text(json.dumps(result, indent=2))
    except BaseException:
        result['error'] = traceback.format_exc()
    Path(args.output).write_text(json.dumps(result, indent=2))
    print('CONNECTION_RESULT '+json.dumps(result),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':
    raise SystemExit(main())
