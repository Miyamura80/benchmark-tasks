"""Compare a pair of isolated native carton hinges under gravity and a bend torque."""
import argparse
import json
import math
import os
from pathlib import Path

from crease import physical_bottom_angle
from scene import layout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True)
    os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
    from isaaclab.app import AppLauncher

    app = AppLauncher(headless=True, device='cpu').app
    try:
        import numpy as np
        import omni.physics.tensors as tensors
        from isaaclab.sim import SimulationCfg, SimulationContext
        from pxr import Gf, UsdGeom, UsdUtils
        from scipy.spatial.transform import Rotation

        from native import export, install_joint

        dt = 1 / 240
        start_angle = 135.
        max_bend_torque = .008  # N m, applied only during the deliberate bend
        sim = SimulationContext(SimulationCfg(dt=dt, device='cpu', create_stage_in_memory=True))
        stage = sim.stage
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        source = export(args.out / 'flap.usda', 'west_bottom', west_finger_notch=True)
        hinge = next(h for h in layout()[1] if h['b'] == 'west_bottom')
        old_hinge = dict(hinge, damping=.001, max_torque=None)
        body_paths = {}
        wall_frame = Rotation.from_euler('x', 180, degrees=True)
        flap_frame = wall_frame * Rotation.from_euler('x', start_angle, degrees=True)
        pin_locations = {}
        for label, properties in (('Original', old_hinge), ('Revised', hinge)):
            parent = f'/World/{label}'
            flap_path = parent + '/west_bottom'
            wall_z = .8
            x = -.25 if label == 'Original' else .25
            flap = UsdGeom.Xform.Define(stage, flap_path)
            flap.GetPrim().GetReferences().AddReference(source)
            anchor = np.array((x, 0, wall_z)) + wall_frame.apply(hinge['anchor_a'])
            pin_locations[label] = anchor
            center = anchor - flap_frame.apply(hinge['anchor_b'])
            flap.AddTranslateOp().Set(Gf.Vec3d(*center))
            xq, yq, zq, wq = flap_frame.as_quat()
            flap.AddOrientOp().Set(Gf.Quatf(wq, xq, yq, zq))
            joint = install_joint(stage, parent, properties, sealed=False)
            joint.GetBody0Rel().SetTargets([])
            joint.GetLocalPos0Attr().Set(Gf.Vec3f(*anchor))
            center_angle = math.radians(180 + hinge['frame_center']) / 2
            joint.GetLocalRot0Attr().Set(Gf.Quatf(math.cos(center_angle), math.sin(center_angle), 0., 0.))
            body_paths[label] = flap_path

        sim.reset()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        world = tensors.create_simulation_view('numpy', stage_id=stage_id)
        bodies = {label: world.create_rigid_body_view(path) for label, path in body_paths.items()}
        indices = np.array([0], dtype=np.int32)
        zero_velocity = np.zeros((1, 6), dtype=np.float32)
        # Initialize both test articles at the same angle after the engine's warm-up steps.
        for label, body in bodies.items():
            center = pin_locations[label] - flap_frame.apply(hinge['anchor_b'])
            pose = np.array([[*center, *flap_frame.as_quat()]], dtype=np.float32)
            body.set_transforms(pose, indices)
            body.set_velocities(zero_velocity, indices)
        rows = []
        for step in range(481):
            row = {'time_s': step * dt}
            for label, body in bodies.items():
                actual = body.get_transforms()[0]
                rotation = Rotation.from_quat(actual[3:])
                pin = actual[:3] + rotation.apply(hinge['anchor_b'])
                row[label.lower() + '_anchor_error_m'] = float(np.linalg.norm(pin - pin_locations[label]))
                relative = wall_frame.inv() * rotation
                qx, qy, qz, qw = relative.as_quat()
                raw = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)))
                row[label.lower() + '_deg'] = physical_bottom_angle(raw)
            row['bend_active'] = 240 <= step < 408
            speed = float(bodies['Revised'].get_velocities()[0, 3])
            torque = min(max_bend_torque, max(0., .025*(math.radians(75.) - speed)))
            row['applied_torque_nm'] = torque if row['bend_active'] else 0.
            rows.append(row)
            if row['bend_active']:
                torques = np.array([[torque, 0, 0]], dtype=np.float32)
                bodies['Revised'].apply_forces_and_torques_at_position(None, torques, None, indices, True)
            sim.step(render=False)
        first, released, bent = rows[0], rows[240], rows[408]
        for label in ('original', 'revised'):
            assert abs(first[label + '_deg'] - start_angle) < .05
            assert max(row[label + '_anchor_error_m'] for row in rows) < .0001
        assert released['original_deg'] - first['original_deg'] > 90.
        assert 0. < released['revised_deg'] - first['revised_deg'] < 12.
        assert bent['revised_deg'] - released['revised_deg'] > 30.
        (args.out / 'measurements.json').write_text(json.dumps(rows, separators=(',', ':')) + '\n')
        print(json.dumps({'initial': first, 'after_gravity': released, 'after_bend': bent, 'final': rows[-1]}), flush=True)
    finally:
        app.close()


if __name__ == '__main__':
    main()
