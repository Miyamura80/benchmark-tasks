"""Route the empty gripper around the measured garment and inspection frame."""

import mujoco as mj
import numpy as np
from scipy.optimize import minimize


class FreeMotion:
    def __init__(self, model, data, arm):
        self.model = model
        self.work = mj.MjData(model)
        self.work.qpos[:] = data.qpos
        self.arm = arm
        prefix = arm.side + '_'
        self.robot = np.array([
            model.geom(i).name.startswith(prefix) and '_contact_' in model.geom(i).name
            for i in range(model.ngeom)
        ])
        self.rng = np.random.default_rng(41)
        self.tests = 0

    def robot_contacts(self, data):
        geoms = data.contact.geom
        return np.any(self.robot[np.maximum(geoms, 0)] & (geoms >= 0), axis=1)

    def route_contact(self, data):
        """Abort empty-hand travel if a native contact carries over 3 N."""
        contacts = data.contact
        loaded = self.robot_contacts(data) & (contacts.dist < 0) & (contacts.efc_address >= 0)
        force = np.empty(6)
        for index in np.flatnonzero(loaded):
            mj.mj_contactForce(self.model, data, int(index), force)
            if force[0] > 3.:
                names = [self.model.geom(g).name if g >= 0 else 'briefs' for g in contacts.geom[index]]
                return f'Unexpected {force[0]:.1f} N free-motion contact between {names[0]} and {names[1]}'
        return None

    def clear(self, joints):
        """Check one hypothetical posture on a separate MjData."""
        self.tests += 1
        self.work.qpos[self.arm.positions] = joints
        mj.mj_kinematics(self.model, self.work)
        mj.mj_comPos(self.model, self.work)
        mj.mj_flex(self.model, self.work)
        mj.mj_collision(self.model, self.work)
        return not np.any(self.robot_contacts(self.work) & (self.work.contact.dist < -.00005))

    def separate(self, joints):
        """Find a short normal release from light contact before free travel."""
        if self.clear(joints):
            return None
        contacts = self.work.contact
        indexes = np.flatnonzero(self.robot_contacts(self.work) & (contacts.dist < 0))
        jacobian = np.zeros((3, self.model.nv))
        rows, gaps = [], []
        for index in indexes:
            contact = contacts[index]
            derivative = np.zeros(6)
            for side, geom in enumerate(contact.geom):
                if geom < 0 or not self.robot[geom]:
                    continue
                body = int(self.model.geom_bodyid[geom])
                mj.mj_jac(self.model, self.work, jacobian, None, contact.pos, body)
                sign = 1 if side == 1 else -1
                derivative += sign * (contact.frame[:3] @ jacobian[:, self.arm.dofs])
            if np.linalg.norm(derivative) > .0001:
                rows.append(derivative)
                gaps.append(.004 - contact.dist)
        if not rows:
            raise RuntimeError('No movable contact normal for the empty hand')
        normals = np.stack(rows)
        gaps = np.array(gaps)
        bounds = list(zip(np.maximum(-.10, self.arm.low - joints),
                          np.minimum(.10, self.arm.high - joints)))
        result = minimize(lambda v: v @ v, np.zeros(6), jac=lambda v: 2*v,
                          bounds=bounds, method='SLSQP',
                          constraints=[{'type': 'ineq', 'fun': lambda v: normals @ v - gaps,
                                        'jac': lambda _: normals}])
        if not result.success:
            raise RuntimeError('Cannot separate the empty jaw along its contact normals')
        return joints + result.x

    def edge(self, start, goal):
        steps = max(1, int(np.ceil(np.max(abs(goal - start)) / .022)))
        return all(self.clear(start + (goal - start) * k / steps) for k in range(1, steps + 1))

    def extend(self, tree, target):
        nodes, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.array(nodes) - target, axis=1)))
        delta = target - nodes[nearest]
        distance = np.linalg.norm(delta)
        reached = distance <= .40
        point = target if reached else nodes[nearest] + .40 * delta / distance
        if not self.edge(nodes[nearest], point):
            return None, False
        nodes.append(point.copy())
        parents.append(nearest)
        return len(nodes) - 1, reached

    @staticmethod
    def branch(tree, index):
        nodes, parents = tree
        route = []
        while index >= 0:
            route.append(nodes[index])
            index = parents[index]
        return route[::-1]

    def shorten(self, route):
        simple = [route[0]]
        at = 0
        while at < len(route) - 1:
            end = len(route) - 1
            while end > at + 1 and not self.edge(route[at], route[end]):
                end -= 1
            simple.append(route[end])
            at = end
        return simple

    def path(self, start, goal):
        if not self.clear(start):
            raise ValueError('Starting arm already touches the workspace')
        if not self.clear(goal):
            raise ValueError('Free-space approach goal intersects the workspace')
        if self.edge(start, goal):
            return [start, goal]

        # Bidirectional RRT-Connect. Each edge is tested in both robot and
        # textile geometry; only the resulting joint positions drive the arms.
        trees = [([start], [-1]), ([goal], [-1])]
        for k in range(1800):
            front, back = trees[k % 2], trees[1 - k % 2]
            target = self.rng.uniform(self.arm.low, self.arm.high)
            if k % 5 == 0:
                target = back[0][self.rng.integers(len(back[0]))]
            front_index, _ = self.extend(front, target)
            if front_index is None:
                continue
            while True:
                back_index, reached = self.extend(back, front[0][front_index])
                if back_index is None:
                    break
                if reached:
                    route = self.branch(front, front_index) + self.branch(back, back_index)[-2::-1]
                    return self.shorten(route[::-1] if k % 2 else route)
        raise RuntimeError('Could not route the free hand around the fixture')
