"""Physical finishing pass for one remaining seam cell."""
import numpy as np


def missing_cells(coverage):
    return [
        {'id': index, 'group': group, 'local_xyz_m': list(center)}
        for index, (group, center) in enumerate(coverage.cells)
        if index not in coverage.cut
    ]


def finishing_phases(probe, cell):
    x, y, _ = cell['local_xyz_m']
    if cell['group'] != 'seam' or not np.isfinite([x, y]).all():
        return None
    if abs(x) >= .1089:
        # Traverse the final millimetre of seam crosswise from its end fold.
        # This keeps the blade out of the end wall during a lengthwise cut.
        front = x < 0.
        group = 'front_fold' if front else 'back_fold'
        yaw = probe.front_yaw if front else -np.pi / 2
        depth = probe.fold_film_depth
        start = (x, y - .006, depth)
        target = (x, y, depth)
    else:
        group = 'seam'
        yaw = probe.seam_yaw
        depth = probe.seam_depth
        stop_x = float(np.clip(x, -.107, .1065))
        # Use the measured interior corridor from the ordinary seam stroke.
        approach_x = stop_x + (.006 if stop_x < 0. else -.006)
        start = (approach_x, y, depth)
        target = (stop_x, y, depth)
    stem = f"finish_cell_{cell['id']}_{group}"
    incoming = next((p[2] for p in reversed(probe.phases[:probe.index])
                     if p[2] is not None), -np.pi/2)
    phases = []
    if group == 'seam':
        staging = (0., -.05, -.04)
        phases.extend([(stem+'_staging', staging, incoming), (stem+'_orient', staging, yaw)])
    else:
        phases.append((stem+'_orient', None, yaw))
    phases.extend([
        (stem+'_hover', (*start[:2], -.020), yaw),
        (stem+'_lower', start, yaw),
        (stem+'_cut', target, yaw),
        (stem+'_lift', (*target[:2], -.020), yaw),
    ])
    return phases
