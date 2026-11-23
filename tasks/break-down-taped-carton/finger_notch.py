"""Die-cut opening in the free edge of the west inner flap."""
NOTCH_WIDTH = .028
NOTCH_DEPTH = .010


def board_sections(size):
    width, depth, thickness = size
    shoulder = (width - NOTCH_WIDTH) / 2
    return (
        ('back', (width, depth - NOTCH_DEPTH, thickness), (0., NOTCH_DEPTH / 2, 0.)),
        ('left', (shoulder, NOTCH_DEPTH, thickness), (-(width + NOTCH_WIDTH) / 4, (NOTCH_DEPTH - depth) / 2, 0.)),
        ('right', (shoulder, NOTCH_DEPTH, thickness), ((width + NOTCH_WIDTH) / 4, (NOTCH_DEPTH - depth) / 2, 0.)),
    )


def hook_obstacles(hook, *, lifting=False):
    obstacles = {name: (*hook.pose(name), size) for name, size in hook.sizes.items()}
    if not hook.finger_notch:
        return obstacles, (hook.flap + '_bottom',)
    position, rotation, size = obstacles.pop('west_bottom')
    for name, section, local_center in board_sections(size):
        obstacles['west_bottom_' + name] = (position + rotation @ local_center, rotation, section)
    targets = ('west_bottom_back', 'west_bottom_left', 'west_bottom_right') if lifting else ('west_bottom_back',)
    return obstacles, targets
