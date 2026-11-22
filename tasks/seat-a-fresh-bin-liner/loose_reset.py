"""Hands-free, zero-sector starting conditions for seating a liner around a rim."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from cuff_walk import Pinches, aperture_samples, edge_lengths, opening_count, rim_sectors
from probe import SEGMENTS, STEP, UNIT, WALL_HEIGHT, WALL_LAYERS, ease, model


def turn_hem_inward(m, d, hem_radius=.110, seam_radius=.132, scallop=1., angle_deg=0.):
    """Fold the free 41 mm hem up/in before releasing it under gravity.

    Rest lengths remain those of the same bag in every configuration. Alternate
    small pleats accommodate the full hem circumference at a smaller radius.
    """
    reference = d.flexvert_xpos.copy()
    next_ring = reference[SEGMENTS:2*SEGMENTS]
    delta_angle = 2 * np.pi / SEGMENTS
    rest_radius = np.linalg.norm(reference[0, :2])
    edge = 2 * rest_radius * np.sin(delta_angle/2)
    # For alternating radii r+a and r-a the neighboring hem edges remain
    # length e when e² = 4*(r*sin(theta/2))² + 4*(a*cos(theta/2))².
    pleat = np.sqrt(max(0., edge**2-(2*hem_radius*np.sin(delta_angle/2))**2))
    pleat *= scallop/(2*np.cos(delta_angle/2))
    radii = hem_radius + pleat * (-1.)**np.arange(SEGMENTS)
    radial_offset = radii - seam_radius
    length = next_ring[:, 2] - reference[:SEGMENTS, 2]
    height = np.sqrt(np.maximum(length**2-radial_offset**2, 0.))
    target = reference.copy()
    target[SEGMENTS:2*SEGMENTS, :2] = seam_radius*UNIT
    target[:SEGMENTS,:2] = radii[:,None]*UNIT
    target[:SEGMENTS,2] = next_ring[:,2] + height
    if angle_deg:
        angle = np.deg2rad(angle_deg)
        rot = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        target[:,:2] = target[:,:2] @ rot.T
    bodies = m.flex_vertbodyid
    joint = m.body_jntadr[bodies]
    indices = m.jnt_qposadr[joint][:, None] + np.arange(3)
    axes = m.jnt_axis[joint[:,None]+np.arange(3)]
    d.qpos[indices] += np.einsum('bij,bj->bi', axes, target-reference)
    mujoco.mj_forward(m,d)
    return reference


def metrics(points, reference_lengths, samples):
    hem = points[:SEGMENTS]
    norms = np.linalg.norm(hem[:,:2],axis=1)
    return dict(sectors=int(rim_sectors(points).sum()),
                opening=opening_count(points,samples)/len(samples),
                strain=float(np.max(np.abs(edge_lengths(points)/reference_lengths-1))),
                hem_radius_m=[float(norms.min()),float(norms.max())],
                hem_z_m=[float(hem[:,2].min()),float(hem[:,2].max())],
                bottom_centre_z_m=float(points[-1,2]))


def passive(m, d, puck_v, seconds):
    for _ in range(round(seconds/m.opt.timestep)):
        d.qfrc_applied[puck_v+2] = .981
        mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d)



def pleated_tube(xml, crown_radius, wall_peak=.112, unfolded_radius=.134, hem_z=.279):
    """Use an isometric zigzag around each ring to fit a full-width bag.

    For a regular 32-sided tube the free radius controls the material width.
    The peak radius prescribes where alternate creases touch the basket.
    Every ring has the same perimeter; there is no bending-rest angle.
    """
    alpha = 2*np.pi/SEGMENTS
    side_length = 2*unfolded_radius*np.sin(alpha/2)
    highs = [crown_radius, wall_peak, wall_peak, wall_peak, wall_peak, wall_peak, .084]
    heights = [hem_z, .251, .198, .142, .085, .032, .023]
    rings = []
    for high,z in zip(highs,heights):
        low = high*np.cos(alpha) - np.sqrt(side_length**2 - (high*np.sin(alpha))**2)
        radii = np.where(np.arange(SEGMENTS)%2 == 0, high, low)
        rings.append(np.column_stack([radii[:,None]*UNIT, np.full(SEGMENTS,z)]))
    points = np.vstack([*rings, [0.,0.,.023]])
    tree = ET.fromstring(xml)
    tree.find("worldbody/flexcomp").set("point"," ".join(f"{x:.9g}" for x in points.ravel()))
    return ET.tostring(tree,encoding="unicode")


def four_point_preparation(m, d, puck_v, reference, inward_radius, settle_s, offset=0):
    """Offline setup only: briefly lift and turn four equally spaced hem points."""
    timestep = m.opt.timestep
    picks = (np.arange(4) * SEGMENTS // 4 + offset) % SEGMENTS
    pinches = Pinches(m, m.flex_vertbodyid, .22)
    rest_lengths = edge_lengths(reference)
    peak = 0.
    for tick in range(round((1.4 + settle_s) / timestep)):
        time = tick * timestep
        d.xfrc_applied[:] = 0
        d.qfrc_applied[puck_v+2] = .981
        if time <= 1.4:
            for hand, vertex in enumerate(picks):
                raised = np.r_[.142 * UNIT[vertex], .312]
                inner = np.r_[inward_radius * UNIT[vertex], .299]
                target = (ease(reference[vertex], raised, time/.60) if time < .60
                          else ease(raised, inner, (time-.60)/.60))
                pinches.hold(d, hand, int(vertex), target)
        mujoco.mj_step(m,d)
        if tick % 10 == 0:
            peak = max(peak, float(np.max(np.abs(edge_lengths(d.flexvert_xpos)/rest_lengths-1))))
    return peak


def prepared_scene(stock_file=None, *, film_mass=.0036, timestep=.00125, settle_s=.7,
                   hem_radius=.110, seam_radius=.132, scallop=1., angle_deg=0.,
                   stiff_pads=False, style="turned", crown_radius=.123, wall_peak=.112,
                   unfolded_radius=.134, hem_z=.279, prep_radius=.108, offset=0):
    def setup(xml):
        if style == "pleated":
            return pleated_tube(xml,crown_radius,wall_peak,unfolded_radius,hem_z)
        return xml
    source, state, puck, v, puck_geom = model(True, setup, film_mass=film_mass, timestep=timestep)
    ref = (state.flexvert_xpos.copy() if style in ("pleated","four_pull") else
           turn_hem_inward(source,state,hem_radius,seam_radius,scallop,angle_deg))
    rest = edge_lengths(ref)
    initial_strain = float(np.max(abs(edge_lengths(state.flexvert_xpos)/rest-1)))
    peak = initial_strain
    if style == "four_pull":
        peak = four_point_preparation(source,state,v,ref,prep_radius,settle_s,offset)
    for tick in range(0 if style == "four_pull" else round(settle_s/timestep)):
        state.qfrc_applied[v+2]=.981
        mujoco.mj_step(source,state)
        if tick % 10 == 0:
            peak=max(peak,float(np.max(abs(edge_lengths(state.flexvert_xpos)/rest-1))))
    mujoco.mj_forward(source,state)
    if stock_file is None:
        return source,state,puck,v,puck_geom,ref,peak
    from native_cuff import scene_xml
    m,d,puck,v,puck_geom=model(True,lambda xml:scene_xml(setup(xml),stock_file,stiff_pads),
                               film_mass=film_mass,timestep=timestep)
    assert np.array_equal(source.jnt_qposadr,m.jnt_qposadr[:source.njnt])
    assert np.array_equal(source.jnt_dofadr,m.jnt_dofadr[:source.njnt])
    d.qpos[:source.nq]=state.qpos
    d.qvel[:source.nv]=state.qvel
    mujoco.mj_forward(m,d)
    return m,d,puck,v,puck_geom,ref,peak


def assess(hold_s=3.5, **kwargs):
    m,d,puck,v,g,ref,peak=prepared_scene(**kwargs)
    lengths=edge_lengths(ref)
    samples=aperture_samples()
    reports = []
    minimum_opening = 1.
    maximum_sectors = 0
    minimum_sectors = 12
    longest_zero = 0.
    consecutive_zero = 0.
    worst_strain = 0.
    initial = d.flexvert_xpos.copy()
    max_drift = 0.
    frozen = None
    dt = m.opt.timestep
    for tick in range(round(hold_s/dt)+1):
        points = d.flexvert_xpos
        sectors = int(rim_sectors(points).sum())
        aperture = opening_count(points,samples)/len(samples)
        minimum_opening = min(minimum_opening, aperture)
        minimum_sectors = min(minimum_sectors, sectors)
        maximum_sectors = max(maximum_sectors, sectors)
        consecutive_zero = consecutive_zero+dt if sectors == 0 else 0.
        longest_zero = max(longest_zero,consecutive_zero)
        if sectors and frozen is None:
            frozen = tick*dt
        if tick % round(.05/dt) == 0 or tick == round(hold_s/dt):
            row = metrics(points,lengths,samples)
            hem = points[:SEGMENTS]
            hem_radius = np.linalg.norm(hem[:,:2],axis=1)
            row["exterior_hem_vertices"] = int(np.sum(hem_radius>=.125))
            row["exterior_at_least_3_6_9_mm_below"] = [
                int(np.sum((hem_radius>=.125) & (hem[:,2]<=WALL_HEIGHT-depth))) for depth in (.003,.006,.009)]
            crown_contacts = []
            for contact in d.contact:
                if 0 not in contact.flex or max(contact.geom) < 0 or contact.efc_address < 0:
                    continue
                name = m.geom(max(contact.geom)).name
                if name.startswith("lip") and contact.dist <= .00065:
                    crown_contacts.append(round(int(name[3:])*12/SEGMENTS)%12)
            row["crown_contact_sectors"] = len(set(crown_contacts))
            row["min_shell_z_m"] = float(points[:,2].min())
            row["max_hem_drift_m"] = float(np.linalg.norm(hem-initial[:SEGMENTS],axis=1).max())
            row["time_s"] = round(tick*dt,5)
            max_drift = max(max_drift,row["max_hem_drift_m"])
            worst_strain = max(worst_strain,row["strain"])
            reports.append(row)
        if tick < round(hold_s/dt):
            d.qfrc_applied[v+2] = .981
            mujoco.mj_step(m,d)
    return dict(config=kwargs,preparation_peak_strain=peak,hold_s=hold_s,
                passive_min_opening=minimum_opening,passive_min_sectors=minimum_sectors,
                passive_max_sectors=maximum_sectors,first_seated_s=frozen,
                longest_zero_s=min(longest_zero,hold_s),max_sampled_hem_drift_m=max_drift,
                max_sampled_passive_strain=worst_strain,passive=reports,
                warnings=[w.number for w in d.warning])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--hold-s',type=float,default=3.5)
    p.add_argument('--prep-radius',type=float,default=.108)
    p.add_argument('--offset',type=int,default=0)
    p.add_argument('--style',choices=('turned','pleated','four_pull'),default='turned')
    p.add_argument('--crown-radius',type=float,default=.123)
    p.add_argument('--wall-peak',type=float,default=.112)
    p.add_argument('--unfolded-radius',type=float,default=.134)
    p.add_argument('--hem-z',type=float,default=.279)
    p.add_argument('--hem-radius',type=float,default=.110)
    p.add_argument('--seam-radius',type=float,default=.132)
    p.add_argument('--scallop',type=float,default=1.)
    p.add_argument('--angle-deg',type=float,default=0.)
    p.add_argument('--settle-s',type=float,default=.7)
    p.add_argument('--film-mass',type=float,default=.0036)
    p.add_argument('--timestep',type=float,default=.00125)
    args=vars(p.parse_args()); out=args.pop('output')
    result=assess(**args)
    out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result))
