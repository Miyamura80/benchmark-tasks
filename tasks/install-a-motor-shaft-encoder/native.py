"""Bridge the task's USD assets and measured PhysX contacts to YAM Arena."""
from math import isfinite

import numpy as np


def substitute_assets(assets, build, post_spawn=None):
    """Return an _ensure_sim wrapper. Build assets after Kit but before PhysX."""
    def ensure(instance, goal):
        from yam_arena import inspect_embodiment as adapter

        if instance._env is not None and goal == instance._goal:
            return
        if adapter._APP is None:
            import atexit
            from isaaclab.app import AppLauncher

            adapter._APP = AppLauncher(headless=True, device=instance.device, enable_cameras=True).app
            atexit.register(adapter._close_app_at_exit)
        import isaaclab.sim as sim
        from yam_arena import scene as sc

        paths = build()
        original = sc.task_object

        def make(spec):
            path = paths.get(spec.name)
            if path is None:
                return original(spec)
            cfg = sim.UsdFileCfg(usd_path=str(path))
            if post_spawn is not None:
                spawn = cfg.func

                def spawn_native(prim_path, cfg, translation=None, orientation=None, **kwargs):
                    prim = spawn(prim_path, cfg, translation=translation, orientation=orientation, **kwargs)
                    post_spawn(spec, prim)
                    return prim

                cfg.func = spawn_native
            return sc.Object(
                name=spec.name,
                object_type=sc.ObjectType.RIGID if spec.free else sc.ObjectType.BASE,
                spawner_cfg=cfg,
                initial_pose=sc.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=sc._yaw_xyzw(spec.yaw)),
            )

        if paths.keys() != assets:
            raise ValueError("Native asset names must match task placeholders")
        sc.task_object = make
        try:
            ensure.original(instance, goal)
        finally:
            sc.task_object = original

    return ensure


def asset(path, mass=None):
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Object").GetPrim()
    stage.SetDefaultPrim(root)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(mass)
        report = PhysxSchema.PhysxContactReportAPI.Apply(root)
        report_threshold = getattr(report, "CreateThresholdAttr", None)
        if report_threshold is None:
            report_threshold = report.CreatePhysxContactReportThresholdAttr
        report_threshold().Set(0.)
    return stage


def collider(shape, friction=None, contact_offset=.0005):
    from pxr import PhysxSchema, UsdPhysics, UsdShade
    from pxr import Sdf

    prim = shape.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(contact_offset)
    api.CreateRestOffsetAttr(0.)
    if friction is not None:
        stage = prim.GetStage()
        material = UsdShade.Material.Define(stage, Sdf.Path('/Object/GripMaterial'))
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(friction)
        physics.CreateDynamicFrictionAttr(friction)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')


class Contacts:
    def __init__(self, relevant):
        from omni.physx import get_physx_simulation_interface

        self.relevant = set(relevant)
        self._names = {}
        self._contacts = []
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self._receive)

    def _name(self, encoded):
        from pxr import PhysicsSchemaTools

        if encoded not in self._names:
            self._names[encoded] = str(PhysicsSchemaTools.intToSdfPath(encoded)).split("/")
        return self._names[encoded]

    def _receive(self, headers, data):
        for header in headers:
            if not header.num_contact_data:
                continue
            components = self._name(header.collider0) + self._name(header.collider1)
            if not self.relevant.intersection(components):
                continue
            accepted_shapes = self.relevant | {
                "tip_left", "tip_right", "gripper", "CeramicSurface", "MiddleNail", "middle_finger_proxy"
            }
            pair = frozenset(accepted_shapes.intersection(components))
            for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                contact = data[i]
                impulse = float(np.linalg.norm(contact.impulse))
                if isfinite(impulse) and impulse > 0.:
                    self._contacts.append((pair, np.array(contact.position), impulse))

    def take(self):
        contacts, self._contacts = self._contacts, []
        return contacts

def root_states(embodiment, names):
    """Return measured body-root poses for compound USD assets."""
    from dataclasses import replace
    from yam_arena.scene import object_states

    objects = object_states(embodiment._env, names)
    world = embodiment._base.arena_world
    origin = embodiment._base.scene.env_origins[0].detach().cpu().numpy()
    return {
        name: replace(state, pos=world.get_pose_w(name)[0, :3].detach().cpu().numpy() - origin)
        for name, state in objects.items()
    }

