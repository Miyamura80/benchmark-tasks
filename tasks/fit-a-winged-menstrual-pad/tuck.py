"""Seat the near wing from below and move both unloaded stock grippers clear."""
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from free_motion import FreeMotion
from robot import pose_frame


def finish_near_fold(trial, material):
    model, data = trial.model, trial.data
    names = [model.geom(i).name for i in range(model.ngeom)]
    robot = np.array([name.startswith(('left_', 'right_')) and '_contact_' in name for name in names])
    hardware = ('bearing_', 'inspection_axle', 'axle_', 'outer_rim', 'crossmember',
                'rear_rest', 'side_clip', 'corner_clip', 'table', 'shelf')
    fixture = np.array([name.startswith(hardware) for name in names])
    wrench = np.zeros(6)

    def guard(state):
        pairs = state.contact.geom
        valid = pairs >= 0
        indices = np.maximum(pairs, 0)
        arms = robot[indices] & valid
        bench = fixture[indices] & valid
        contacts = ((arms[:, 0] & bench[:, 1]) | (arms[:, 1] & bench[:, 0]))
        for index in np.flatnonzero(contacts & (state.contact.efc_address >= 0)):
            mj.mj_contactForce(model, state, index, wrench)
            if wrench[0] > 2.0:
                pair = [names[i] for i in pairs[index]]
                return f'Rigid contact {pair} reached {wrench[0]:.2f} N'

    def move(side, label, point, rotation, opening, duration):
        trial.move(label, {side: (np.array(point), rotation, opening)}, duration, strict=False, guard=guard)
        residual = np.linalg.norm(trial.arms[side].pose(data)[0] - point)
        if residual > .012:
            raise RuntimeError(f'{label}: {side} tracking error {1000*residual:.1f} mm')

    def park(side, point):
        arm = trial.arms[side]
        direction = 1 if side == 'left' else -1
        rotation = pose_frame((direction, 0, 0))
        start = data.qpos[arm.positions].copy()
        opening = trial.last[side][-1]
        plan = FreeMotion(model, data, arm)
        for index in range(4):
            separated = plan.separate(start)
            if separated is None:
                break
            pos, frame = arm.forward(separated)
            trial.move(f'Separate {side} {index+1}', {side: (pos, frame, opening)}, .2,
                       joint_targets={side: separated}, guard=plan.route_contact)
            start = data.qpos[arm.positions].copy()
            plan = FreeMotion(model, data, arm)
        for seed in (start, None):
            goal, error = arm.solve(point, rotation, seed)
            if error < .004 and plan.clear(goal):
                break
        else:
            raise RuntimeError(f'Cannot find a clear parking pose for {side}')
        route = plan.path(start, goal)
        for index, (a, b) in enumerate(zip(route, route[1:])):
            trial.move(f'Park {side} {index+1}', {side: (*arm.forward(b), opening)},
                       max(.45, max(abs(b-a))/.9), joint_targets={side: b}, guard=plan.route_contact)
        move(side, 'Open parked '+side, point, rotation, .014, .4)

    yaw, pitch, roll = np.radians([65., -10., 200.])
    heading = np.array([np.cos(pitch)*np.cos(yaw), -np.cos(pitch)*np.sin(yaw), np.sin(pitch)])
    face = pose_frame(heading) @ Rotation.from_rotvec([0, 0, roll]).as_matrix()
    tip = face @ np.array([-.005, -.0034, .0095])
    film = data.geom('near_glue')
    move('left', 'Align the distal fingertip beneath the wing',
         film.xpos - film.xmat.reshape(3, 3)[:, 2]*.0025 - tip, face, 0, .6)
    for index in range(14):
        film = data.geom('near_glue')
        cloth = material.textile(data.flexvert_xpos, film.xpos[:2])
        if cloth is None:
            raise RuntimeError('Near adhesive is outside the gusset')
        gap = cloth[0] - film.xpos[2]
        advance = np.clip(gap - .0007, .001, .0045)
        point = film.xpos-tip + [0, -2.5*.001, advance]
        move('left', f'Seat the near wing {index+1}', point, face, 0, .23)
        normal_force = 0.
        for ci, contact in enumerate(data.contact):
            if film.id in contact.geom and material.adh.flex in contact.flex and contact.efc_address >= 0:
                mj.mj_contactForce(model, data, ci, wrench)
                normal_force += wrench[0]
        if normal_force >= .03:
            move('left', 'Hold the near wing against the fabric', point, face, 0, .20)
            break

    p, rotation = trial.arms['left'].pose(data)
    move('left', 'Ease near pressure', p+[0, 0, -.003], rotation, 0, .5)
    p, rotation = trial.arms['right'].pose(data)
    move('right', 'Ease the holding jaws', p, rotation, .004, .5)
    move('right', 'Open the holding jaws', p, rotation, .014, .4)
    park('left', np.array([-.18, .31, .36]))
    p, rotation = trial.arms['right'].pose(data)
    direction = np.array([0, np.cos(np.radians(4)), -np.sin(np.radians(4))])
    move('right', 'Withdraw the open holding hand', p-.045*direction, rotation, .014, .75)
    park('right', np.array([.18, -.17, .36]))
