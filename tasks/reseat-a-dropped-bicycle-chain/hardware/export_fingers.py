"""Export the two stock CAD fingers in the closed left-TCP frame, in mm."""
import struct
from pathlib import Path
import mujoco
import numpy as np
from dream_examples.yam_mujoco.yam_env import build_xml

model = mujoco.MjModel.from_xml_string(build_xml())
data = mujoco.MjData(model)
mujoco.mj_kinematics(model, data)
tcp = data.body("left_grasp_site")
for side in ("left", "right"):
    geom = int(model.body("left_tip_" + side).geomadr[0])
    mesh = int(model.geom_dataid[geom])
    start = model.mesh_vertadr[mesh]
    vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
    world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    points_mm = (world - tcp.xpos) @ tcp.xmat.reshape(3, 3) * 1000
    start = model.mesh_faceadr[mesh]
    faces = model.mesh_face[start:start + model.mesh_facenum[mesh]]
    with Path(__file__).with_name(f"stock-{side}.stl").open("wb") as out:
        out.write(b"YAM stock finger in closed TCP frame".ljust(80, b"\0"))
        out.write(struct.pack("<I", len(faces)))
        for triangle in points_mm[faces]:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            norm = np.linalg.norm(normal)
            if norm:
                normal /= norm
            out.write(struct.pack("<12fH", *normal, *triangle.ravel(), 0))
