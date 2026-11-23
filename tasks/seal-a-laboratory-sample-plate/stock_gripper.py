"""Stock YAM jaws. The task measures the pinch point near the end of the pads."""

def configure_kinematics():
    import sys
    from yam_arena import constants, ik

    # The stock TCP is 13 mm behind the jaw tips. Pinch the sheet with the
    # distal 4 mm so the lower jaw clears the tray beneath the loose edge.
    constants.TCP_OFFSET = (0.0, 0.0, 0.143)
    ik.TCP_OFFSET = constants.TCP_OFFSET
    if 'yam_arena.runtime' in sys.modules:
        sys.modules['yam_arena.runtime'].TCP_OFFSET = constants.TCP_OFFSET


def install():
    from yam_arena import constants, embodiment

    configure_kinematics()
    constants.GRIPPER['effort'] = 4.0
    original = embodiment.yam_arm_cfg
    if getattr(original, '_plate_solver_settings', False):
        return

    def stock_config(*args, **kwargs):
        config = original(*args, **kwargs)
        config.spawn.articulation_props.solver_position_iteration_count = 64
        config.spawn.articulation_props.solver_velocity_iteration_count = 16
        return config

    stock_config._plate_solver_settings = True
    embodiment.yam_arm_cfg = stock_config
