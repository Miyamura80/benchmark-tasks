"""Finite area contact adhesion between exposed adhesive faces and a textile flex.

Native contact force and orientation determine where a bond is made. The three
vertices of the contacted triangle attach at their measured positions. The
solver transmits forces and can break the bond. There is no target placement.
"""
import mujoco as mj
import numpy as np

from robot import xml


def declare(equality, patches, vertex_ids, flex_name):
    for geom_name, body_name in patches.items():
        for vertex in vertex_ids:
            xml(equality, 'connect', name=f'tack_{geom_name}_{vertex}',
                body1=body_name, body2=f'{flex_name}_{vertex}', anchor=(0, 0, 0),
                active='false', solref='.006 1', solimp='.88 .98 .0004')


class TextileAdhesive:
    def __init__(self, model, patches, vertex_ids, flex_name):
        self.model = model
        self.flex = mj.mj_name2id(model, mj.mjtObj.mjOBJ_FLEX, flex_name)
        self.patches = {model.geom(name).id: name for name in patches}
        self.pool = {(model.geom(name).id, v): model.equality(f'tack_{name}_{v}').id
                     for name in patches for v in vertex_ids}
        start = model.flex_elemdataadr[self.flex]
        nfaces = model.flex_elemnum[self.flex]
        self.faces = model.flex_elem[start:start+3*nfaces].reshape(-1, 3)
        self.vert_start = model.flex_vertadr[self.flex]
        self.held = {}
        self.strong = {g: 0. for g in self.patches}
        self.overload = {}
        self.events = []
        self.wrench = np.zeros(6)

    def update(self, data, exposed, dt, pressure=.04, dwell=.075, limit=.45):
        loads = {}
        for index, contact in enumerate(data.contact):
            if self.flex not in contact.flex or contact.efc_address < 0:
                continue
            textile_side = int(contact.flex[1] == self.flex)
            geom = int(contact.geom[1-textile_side]); face = contact.elem[textile_side]
            if geom not in self.patches or geom in self.held or self.patches[geom] not in exposed or face < 0:
                continue
            normal = contact.frame[:3] * (1 if textile_side == 1 else -1)
            if normal @ data.geom_xmat[geom].reshape(3, 3)[:, 2] < .55:
                continue
            mj.mj_contactForce(self.model, data, index, self.wrench)
            magnitude = self.wrench[0]
            if magnitude <= 0:
                continue
            total, best, best_face = loads.get(geom, (0., 0., None))
            loads[geom] = (total+magnitude, max(best, magnitude), face if magnitude>best else best_face)
        for geom in self.patches.keys()-self.held.keys():
            load, _, face = loads.get(geom, (0., 0., None))
            self.strong[geom] = self.strong[geom]+dt if load >= pressure else 0.
            if self.strong[geom] < dwell:
                continue
            vertices = self.faces[face]
            if any((geom, int(v)) not in self.pool for v in vertices):
                continue
            body = self.model.geom_bodyid[geom]
            joints = []
            for v in vertices:
                joint = self.pool[geom, int(v)]; point = data.flexvert_xpos[self.vert_start+v]
                self.model.eq_data[joint,:3] = data.xmat[body].reshape(3,3).T @ (point-data.xpos[body])
                self.model.eq_data[joint,3:6] = 0.
                data.eq_active[joint] = True
                joints.append(joint)
            self.held[geom] = joints
            self.overload[geom] = 0.
            self.events.append({'time': float(data.time), 'event': 'cloth_bond', 'patch': self.patches[geom],
                                'triangle':int(face), 'press_n':float(load)})
        if not self.held:
            return
        eq = data.efc_type == int(mj.mjtConstraint.mjCNSTR_EQUALITY)
        by_id = np.bincount(data.efc_id[eq], data.efc_force[eq]**2, minlength=self.model.neq)**.5
        for geom, joints in list(self.held.items()):
            peak = float(max(by_id[joints]))
            self.overload[geom] = self.overload[geom]+dt if peak>limit else 0.
            if self.overload[geom] >= .004:
                data.eq_active[joints] = False
                del self.held[geom]
                self.strong[geom] = 0.
                self.events.append({'time':float(data.time),'event':'cloth_bond_broke',
                                    'patch':self.patches[geom],'force_n':peak})
