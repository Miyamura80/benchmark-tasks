"""Check native startup at every 1 ms step, with both arms in their actual reset pose."""
import json
import tempfile

import mujoco as mj
import numpy as np

from drive import Driver
from material import Materials
from robot import DT
from run import prepare
from scene import build


def main():
    model,spec=build()
    with tempfile.TemporaryDirectory() as out:
        trial=Driver(model,Materials(model,spec),out)
        prepare(trial)
        data=trial.data
        edge=model.flex_edge
        peak=1.
        drift=0.
        fixture_contacts=0
        hardware=('bearing_','inspection_axle','outer_rim','crossmember','side_clip','corner_clip','rear_rest_')
        ids={g for g in range(model.ngeom) if model.geom(g).name.startswith(hardware)}
        for k in range(round(1/DT)+1):
            positions=data.flexvert_xpos
            lengths=np.linalg.norm(positions[edge[:,0]]-positions[edge[:,1]],axis=1)
            peak=max(peak,float(np.max(lengths/model.flexedge_length0)))
            drift=max(drift,abs(float(data.joint('inspection_lift').qpos[0])))
            fixture_contacts+=sum(1 for c in data.contact if c.dist<0 and any(int(g) in ids for g in c.geom))
            if k<round(1/DT):
                mj.mj_step(model,data)
        report={'native_duration_s':float(data.time),'peak_edge_to_rest_ratio':peak,
                'peak_frame_drift_mm':drift*1000,'penetrating_fixture_contact_samples':fixture_contacts}
        print(json.dumps(report,indent=2))
        if peak>1.05 or drift>.001 or fixture_contacts:
            raise RuntimeError('Reset moved or loaded the fixture before the task began')


if __name__=='__main__':
    main()
