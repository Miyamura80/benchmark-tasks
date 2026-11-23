"""Lift a cut outer flap with the tapered end of one YAM fingertip."""
from itertools import chain, product

import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from angle_tracking import WALLS
from scene import layout
from finger_notch import NOTCH_DEPTH


class CartonFlapHook:
    aperture = .024
    initial_tilt = np.radians(10.)
    inserted_tilt = np.radians(45.)
    clearance = .029
    depth_below_edge = .0022
    entry_gap = .002

    def __init__(self, env, previous=None, flap='north', *, finger_notch=False):
        self.env = env
        self.flap = flap
        self.notched_carton = finger_notch
        self.finger_notch = finger_notch and flap == 'west'
        self.notch_depth = NOTCH_DEPTH if self.finger_notch else 0.
        self.side = 'left' if flap=='north' else None
        self.palm_sign = 1.
        self.edge_shift = 0.
        self.planned_hover = None
        self.heading_offset = 0.
        self.sliding_entry = False
        self.slide_overlap = .006
        self.tip_corner = None
        self.wrist_roll = None
        self.previous = (env._ready.detach().cpu().numpy().reshape(-1).copy()
                         if previous is None else previous.copy())
        self.failure = None
        self.done = False
        self.index = 0
        self.stage = 'hover'
        self.started = None
        self.clearance_wait_started = None
        self.bias = np.zeros(3)
        self.last_angle = -1.
        self.last_progress = 0.
        self.contact_frame = None
        self.shoulder_offset = 0.
        parts, hinges = layout()
        self.hinge = next(h for h in hinges if h['name'] == flap+'_bottom_hinge')
        self.sizes = {p['name']: p['size'] for p in parts}
        self.depth = self.sizes[flap+'_bottom'][1]

    @staticmethod
    def array(v):
        return (v.torch if hasattr(v, 'torch') else v).detach().cpu().numpy()

    def pose(self, name):
        v = self.array(self.env._base.arena_world.get_pose_w(name))[0]
        return v[:3], quat_xyzw_to_matrix(v[3:])

    def change(self, stage, time, point, joints):
        self.stage = stage
        self.started = time
        self.first_point = point.copy()
        self.first_joints = joints.copy()
        self.bias.fill(0.)

    def edge(self, position, rotation):
        return position+rotation @ np.array((self.edge_shift, -self.depth/2. + self.notch_depth, .0015))

    @staticmethod
    def wrist(tilt, outward, up, palm_sign):
        z = outward*np.sin(tilt) - up*np.cos(tilt)
        x = palm_sign*(outward*np.cos(tilt) + up*np.sin(tilt))
        return np.column_stack((x, np.cross(z, x), z))

    def fingertip(self, finger_joint):
        if self.tip_corner is None:
            return np.array((self.palm_sign*(finger_joint-.002), 0., .0125))
        point = self.tip_corner.copy()
        point[0] += finger_joint + self.aperture
        return point

    def working_wrist(self, tilt, heading, up):
        wrist = self.wrist(tilt, heading, up, self.palm_sign)
        return wrist if self.wrist_roll is None else wrist @ self.wrist_roll

    def hand(self, side):
        name = side+'_robot'
        arm_ids, finger_ids = self.env._ids[name]
        joints = self.array(self.env._base.scene[name].data.joint_pos)[0]
        return joints[arm_ids], joints[finger_ids]

    def flap_contact(self, arm, fingers):
        from west_slide import geometry_model
        from finger_notch import hook_obstacles
        geometry = geometry_model()
        obstacles, targets = hook_obstacles(self, lifting=True)
        return geometry.guard('left', arm, fingers, obstacles,
                              geometry.parts('right', *self.hand('right')),
                              intended=True, intended_targets=targets)

    def select_hand(self, edge, tangent, outward, up):
        if self.flap == 'west':
            from west_slide import plan_slide
            slide = plan_slide(self, edge, tangent, outward, up)
            if slide is not None:
                plan, self.tip_corner, self.wrist_roll, self.slide_overlap = slide
                return plan
            if self.finger_notch:
                return None
        from turnover_safety import Geometry
        geometry = Geometry()
        obstacles = {name: (*self.pose(name), size) for name,size in self.sizes.items()}
        gentle_contact = None
        expected = (self.flap+'_bottom',)
        for side in ('left','right'):
            seed, opening = self.hand(side)
            other_side = 'right' if side=='left' else 'left'
            other_q, other_fingers = self.hand(other_side)
            other = geometry.parts(other_side, other_q, other_fingers)
            base = self.env.rig.bases[side]
            if self.flap in ('east','west'):
                shifts = (.032,-.032,.045,-.045,.018,-.018,0.)
            else:
                shifts = (0.,.016,-.016,.035,-.035)
            profiles = ((55.,55.,.012,.007),(67.,67.,.014,.008),(74.,74.,.017,.010),
                        (10.,59.,.002,.0022),(18.,60.,.004,.0032),(26.,62.,.0054,.0048))
            preferred = []
            if side=='left' and self.flap=='south':
                preferred.append(((26.,62.,.0054,.0048),-.90,-1.,0.,.024))
            elif side=='left' and self.flap=='west':
                preferred.extend((((26.,62.,.010,.0048),1.35,1.,0.,.024),
                                  ((26.,62.,.010,.0048),-1.70,-1.,0.,.012),
                                  ((26.,62.,.010,.0048),1.70,1.,.033,.012)))
            grid = product(profiles,(0.,-.384,.384,-.733,.733,-1.08,1.08),(1.,-1.),shifts,(.024,))
            for profile,yaw,sign,offset,aperture in chain(preferred,grid):
                initial,inserted,gap,depth = profile
                heading = axis_angle_matrix(up,yaw) @ outward
                orientation = self.wrist(np.radians(initial), heading, up, sign)
                quat = ik.matrix_to_quat_xyzw(orientation)
                pad = np.array((sign*(-aperture-.002), 0., .0125))
                target = edge+offset*tangent-gap*outward-depth*up-orientation @ pad
                high,e,r = ik.solve(target+self.clearance*up, quat, seed, base_pos=base, iters=125)
                low,pe,re = ik.solve(target, quat, high, base_pos=base, iters=100)
                preferred_west = self.flap=='west' and (profile,yaw,sign,offset,aperture) in preferred
                if preferred_west and (max(e,pe)>.001 or max(r,re)>.014):
                    low,pe,re = ik.solve(target, quat, seed, base_pos=base, iters=100)
                    high,e,r = ik.solve(target+self.clearance*up, quat, low, base_pos=base, iters=125)
                if max(e, pe)>.001 or max(r, re)>.014:
                    continue
                fingers = np.full(2, -aperture)
                route = [(seed+f*(high-seed), opening+f*(fingers-opening), False)
                         for f in np.linspace(0.,1.,9)]
                route.extend((high+f*(low-high), fingers, True) for f in (.33,.66,1.))
                if initial==inserted:
                    beneath,e,r = ik.solve(target+(gap+.006)*outward, quat, low, base_pos=base, iters=125)
                    if e>.001 or r>.014:
                        continue
                    route.extend((low+f*(beneath-low), fingers, True) for f in (.25,.5,.75,1.))
                smallest = np.inf
                for arm, gap, contact in route:
                    clearance = geometry.guard(side,arm,gap,obstacles,other,
                                               intended=contact,intended_targets=expected)
                    if not clearance['safe'] or clearance['target_gap_m']<-.0015:
                        break
                    smallest = min(smallest, clearance['target_gap_m'])
                else:
                    plan = (side, sign, offset, yaw, profile, aperture, high if preferred_west else None)
                    if smallest>=.0002 or (profile,yaw,sign,offset,aperture) in preferred:
                        return plan
                    if gentle_contact is None:
                        gentle_contact = plan
        return gentle_contact

    def command(self, time):
        _, wall_frame = self.pose(self.flap)
        flap, flap_frame = self.pose(self.flap+'_bottom')
        angles = self.env._base._carton_angle_observer.tracker.angles
        angle = float(angles[WALLS.index(self.flap)])
        up = np.array((0., 0., 1.))
        outward = wall_frame[:, 1].copy()
        outward[2] = 0.
        outward /= np.linalg.norm(outward)
        if self.side is None:
            east_angle = float(angles[WALLS.index('east')])
            if self.finger_notch and 150. <= east_angle < 265.:
                if self.clearance_wait_started is None:
                    self.clearance_wait_started = time
                if time - self.clearance_wait_started < 18.:
                    return self.previous.copy(), [dict(phase='hook_wait_for_east', flap=self.flap,
                                                       angle_deg=angle, east_angle_deg=east_angle)]
            selected = self.select_hand(self.edge(flap, flap_frame), flap_frame[:, 0], outward, up)
            if selected is None:
                self.failure = 'no guarded fingertip access to the '+self.flap+' flap'
                return self.previous.copy(), [dict(phase='hook_unreachable', failure=self.failure, angle_deg=angle)]
            (self.side, self.palm_sign, self.edge_shift, self.heading_offset, profile,
             self.aperture, self.planned_hover) = selected
            initial, inserted, self.entry_gap, self.depth_below_edge = profile
            self.initial_tilt, self.inserted_tilt = np.radians((initial,inserted))
            self.sliding_entry = initial==inserted
        q, fingers = self.hand(self.side)
        base = self.env.rig.bases[self.side]
        tcp, wrist = ik.tcp_pose(q, base)
        finger = 0 if self.palm_sign==1. else 1
        fingertip = self.fingertip(fingers[finger])
        point = tcp+wrist @ fingertip
        edge = self.edge(flap, flap_frame)
        if self.started is None:
            cutter, _ = self.pose('cutter')
            by_pad = np.linalg.norm(tcp[:2]-cutter[:2]) < .090 and tcp[2] < edge[2]+.012
            if by_pad:
                self.pad_lift_tcp = np.r_[tcp[:2], edge[2]+.024]
            self.change('raise_from_pad' if by_pad else 'hover', time, point, q)
            self.last_progress = time
        release_angle = 128.
        if self.flap == 'west' and self.tip_corner is not None:
            release_angle = 110. if self.contact_frame is not None else 120.
        if self.stage == 'lift' and self.contact_frame is not None and angle >= release_angle:
            self.change('retreat', time, point, q)
            self.release_wrist = wrist.copy()
        phase = self.stage
        elapsed = time-self.started
        if self.finger_notch and phase == 'lift' and self.contact_frame is None and angle >= 5.:
            state = self.flap_contact(q, fingers)
            if (state['safe'] and state['target_pair'] == ['tip_left', 'west_bottom_left']
                    and -.0015 <= state['target_gap_m'] <= .0002):
                from west_notch_contact import capture_shoulder
                hinge = flap + flap_frame @ np.asarray(self.hinge['anchor_b'])
                self.contact_frame = capture_shoulder(tcp, wrist, flap, flap_frame,
                                                       hinge, self.sizes['west_bottom'])
        contact_follow = phase == 'lift' and self.contact_frame is not None
        tilt = self.initial_tilt
        if phase == 'insert':
            amount = ik.smoothstep(min(1., elapsed/4.))
            tilt += amount*(self.inserted_tilt-self.initial_tilt)
        elif phase in ('lift', 'retreat'):
            tilt = self.inserted_tilt
        heading = axis_angle_matrix(up, self.heading_offset) @ outward
        desired_wrist = self.working_wrist(tilt, heading, up)
        lower = edge - self.entry_gap*outward - self.depth_below_edge*up
        end_to_tip = self.fingertip(-self.aperture)
        if phase == 'raise_from_pad':
            desired_wrist = grip_rotation()
            target = self.pad_lift_tcp + desired_wrist @ end_to_tip
            duration, following = 5., 'hover'
        elif phase == 'hover':
            target, duration, following = lower+self.clearance*up, 5., 'descend'
        elif phase == 'descend':
            target, duration, following = lower, 4., 'insert'
        elif phase == 'insert':
            # Keep the shaft crossing the seam at the same point while the
            # tip turns under the flap. Sideways translation alone pushes the
            # leading edge and slides the whole carton.
            pivot_height = self.depth_below_edge + .003
            displacement = pivot_height*(np.tan(tilt)-np.tan(self.initial_tilt))
            target, duration, following = lower+displacement*heading, 4., 'lift'
            if self.sliding_entry:
                target = lower + (self.entry_gap+self.slide_overlap)*outward
        elif phase == 'lift':
            lead = 8. if self.flap=='north' or angle>=12. else 3.
            turning = np.radians(min(152., angle+lead) - angle)
            predicted = axis_angle_matrix(wall_frame[:, 0], turning) @ flap_frame
            hinge = flap + flap_frame @ np.array(self.hinge['anchor_b'])
            predicted_centre = hinge-predicted @ np.array(self.hinge['anchor_b'])
            handoff = ik.smoothstep(min(1., max(0., (angle-26.)/58.)))
            # Once the edge is raised, push on the exposed inner face at a
            # shorter lever. This keeps the hand inside its reachable height.
            shift = np.copysign(.030, self.edge_shift)
            beneath = .006 if self.sliding_entry else .0022 if self.flap=='north' else .0044
            if self.flap == 'west' and self.tip_corner is not None and angle > 40.:
                beneath = .0017
                handoff = 0.
            target = predicted_centre + predicted @ np.array((
                self.edge_shift+shift*handoff, -self.depth/2.+self.notch_depth+beneath+.026*handoff, .0002))
            desired_wrist = axis_angle_matrix(wall_frame[:, 0], np.radians(min(28.,max(0.,angle*.42)))) @ desired_wrist
            duration, following = 1., 'retreat' if angle >= release_angle else None
            if angle > self.last_angle + .8:
                self.last_angle, self.last_progress = angle, time
            if time-self.last_progress>5.:
                self.failure = 'tapered fingertip did not advance the measured flap'
        else:
            target, duration, following = self.first_point + .045*up, 3., 'done'
            if self.contact_frame is not None:
                desired_wrist = self.release_wrist
        if contact_follow:
            contact_point, contact_wrist, tool_point = self.contact_frame
            contact_rotation = axis_angle_matrix(wall_frame[:, 0], np.radians(.5)) @ flap_frame
            target = hinge + contact_rotation @ contact_point
            desired_wrist = contact_rotation @ contact_wrist
        blend = ik.smoothstep(min(1., elapsed/duration))
        joint_transit = phase in ('raise_from_pad', 'hover')
        pivoting = phase=='insert' and not self.sliding_entry
        waypoint = target if joint_transit or pivoting or contact_follow else self.first_point + blend*(target-self.first_point)
        if not contact_follow and (not joint_transit or blend == 1.):
            self.bias = np.clip(self.bias+.06*(waypoint-point), -.0025, .0025)
        if contact_follow and self.finger_notch:
            from west_notch_contact import follow_shoulder
            guarded = follow_shoulder(self, hinge, flap_frame, contact_rotation, q, fingers)
            if guarded is None:
                self.failure = 'no clear west-notch shoulder contact step'
                return self.previous.copy(), [dict(phase='hook_lift', failure=self.failure, angle_deg=angle)]
            goal_q, pe, re = guarded
        elif contact_follow:
            from flap_contact_ik import solve_contact
            goal_q, pe, re = solve_contact(target, desired_wrist, tool_point, q, base)
        else:
            tcp_goal = waypoint+self.bias-desired_wrist @ end_to_tip
            goal_q, pe, re = ik.solve(tcp_goal, ik.matrix_to_quat_xyzw(desired_wrist), q,
                                     base_pos=base, iters=110)
        if phase == 'lift' and pe > .004 and not contact_follow:
            for degrees in (27., 16., 38., 48.):
                wrist_goal = self.working_wrist(np.radians(degrees), heading, up)
                p = waypoint+self.bias-wrist_goal @ end_to_tip
                trial, e, r = ik.solve(p, ik.matrix_to_quat_xyzw(wrist_goal), q, base_pos=base, iters=85)
                if e+.08*r < pe+.08*re:
                    goal_q, pe, re = trial, e, r
                if pe < .0015 and re < .025:
                    break
        if phase == 'lift' and self.flap == 'west' and self.tip_corner is not None and not contact_follow:
            from west_slide import guarded_lift
            guarded = guarded_lift(self, q, fingers, goal_q, -.85*flap_frame[:, 1]+.15*flap_frame[:, 2])
            if guarded is None:
                self.failure = 'no clear west-flap lifting step'
                return self.previous.copy(), [dict(phase='hook_lift',failure=self.failure,angle_deg=angle)]
            goal_q = guarded
        if contact_follow and not self.finger_notch:
            from west_slide import guarded_lift
            guarded = guarded_lift(self, q, fingers, goal_q, flap_frame[:, 2])
            if guarded is None:
                self.failure = 'no clear north-flap lifting step'
                return self.previous.copy(), [dict(phase='hook_lift', failure=self.failure, angle_deg=angle)]
            goal_q = guarded
        if joint_transit:
            if phase=='hover' and self.planned_hover is not None:
                goal_q = self.planned_hover
            goal_q = self.first_joints+blend*(goal_q-self.first_joints)
        error = float(np.linalg.norm(point-target))
        timeout = 50. if phase == 'retreat' and self.finger_notch else duration+9.
        if elapsed>(22. if phase=='lift' else timeout):
            self.failure = phase+' did not settle at the measured fingertip target'
        inside = (point-edge) @ outward
        below = (edge-point) @ up
        enough_overlap = .0035 if self.sliding_entry and self.tip_corner is None else .0012
        hooked = phase=='insert' and self.flap!='north' and elapsed>=2. and inside>=enough_overlap and below>=.001
        if self.notched_carton and phase == 'insert' and self.flap == 'north' and elapsed >= 2. and 6. <= angle < 12. and inside > 0. and below >= .001 and pe > .004:
            contact = self.flap_contact(q, fingers)
            hooked = (contact['safe'] and contact['target_pair'][0] == 'tip_left'
                      and -.0015 <= contact['target_gap_m'] <= .0002)
            if hooked:
                from flap_contact_ik import capture_contact
                hinge = flap + flap_frame @ np.asarray(self.hinge['anchor_b'])
                self.contact_frame = capture_contact(point, tcp, wrist, flap, flap_frame,
                                                     hinge, self.sizes['north_bottom'])
        caught_on_descent = phase=='descend' and self.flap!='north' and angle>=24. and error<.008
        if caught_on_descent:
            following = 'lift'
        engaged = caught_on_descent or phase=='insert' and (angle>=12. or hooked)
        tolerance = .004 if phase=='raise_from_pad' else .006 if phase=='retreat' else .00105
        if phase == 'insert' and self.tip_corner is not None and not engaged:
            following = None
        settled = error < tolerance or phase == 'lift'
        if phase == 'retreat' and self.contact_frame is not None and blend == 1. and angle >= 150.:
            contact = self.flap_contact(q, fingers)
            settled = contact['safe'] and contact['target_gap_m'] >= .010
        if phase == 'retreat' and self.finger_notch and angle < 150.:
            settled = False
        if following and (engaged or blend == 1. and settled):
            if following == 'done':
                self.done = angle >= 150.
                self.index = int(self.done)
                if not self.done:
                    self.failure = 'flap did not remain open after the fingertip withdrew'
            else:
                retained_bias = self.bias.copy() if hooked or caught_on_descent else None
                self.change(following, time, point, q)
                if retained_bias is not None:
                    self.bias[:] = retained_bias
                self.last_progress = time
        out = self.previous.copy()
        i = 0 if self.side=='left' else 7
        out[i:i+6] = np.clip(goal_q, out[i:i+6]-.03, out[i:i+6]+.03)
        out[i+6] = self.aperture/abs(FINGER_OPEN)
        out = np.clip(out, self.env._low, self.env._high)
        self.previous = out
        return out, [dict(phase='hook_'+phase, flap=self.flap, side=self.side, heading_offset=self.heading_offset, failure=self.failure, angle_deg=angle,
                          finger_over_edge_m=float(inside), finger_below_edge_m=float(below),
                          position_error=error, ik_error=pe, ik_rotation_error=re,
                          target=target.tolist(), fingertip=point.tolist())]
