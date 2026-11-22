"""Small MuJoCo controls for the mat's 1 N lateral breakaway rule."""
import mujoco as mj
import numpy as np
from demo import brush_releases


def trial(gap_mm, stiffness):
    # A mocap bristle approaches an immobile bonded pellet from the side.
    xml = f"""<mujoco><option gravity="0 0 0"/><worldbody>
    <body name="mat"><geom type="box" size=".05 .05 .001" contype="0" conaffinity="0"/></body>
    <body name="grit" pos="0 0 .02"><freejoint/>
      <geom name="crust" type="sphere" size=".005" mass=".001" contype="3"
            solref="{-stiffness} -100" solimp=".9 .9 .001"/></body>
    <body name="brush" mocap="true" pos="{.010 + gap_mm / 1000} 0 .02">
      <geom name="bristle" type="sphere" size=".005" contype="0" conaffinity="2"
            solref="{-stiffness} -100" solimp=".9 .9 .001"/></body>
    </worldbody><equality><weld body1="mat" body2="grit" name="bond"/></equality></mujoco>"""
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    force = np.zeros(6)
    if data.ncon:
        mj.mj_contactForce(model, data, 0, force)
    events = brush_releases(model, data, {model.geom('bristle').id},
                            {model.geom('crust').id: model.equality('bond').id},
                            model.body('mat').id)
    return int(data.ncon), float(force[0]), bool(events)


if __name__ == '__main__':
    import json
    results = dict(proximity=trial(0.5, 1e6), weak=trial(-0.1, 1.8e6),
                   loaded=trial(-0.1, 2e6))
    print(json.dumps(results, indent=2))
    assert results['proximity'] == (0, 0, False)
    assert results['weak'][0] and 0 < results['weak'][1] < 1 and not results['weak'][2]
    assert results['loaded'][1] >= 1 and results['loaded'][2]
