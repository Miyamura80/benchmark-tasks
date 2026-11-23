"""Catch a parked cutter being covered by the planned inverted carton."""
import unittest,math
import numpy as np
from scene import layout
from config import CENTER,TOP,CUTTER_START,CUTTER_PAD_START
from model import WIDTH,HEIGHT
from check_turnover import parked_cutter
from yam_arena.urdf import quat_xyzw_to_matrix
from turnover_safety import Geometry
from turnover_targets import folded_flap_target, tip_vertices, wall_target
class Tests(unittest.TestCase):
 def test_live_outer_flap_target_balances_pad_contact_after_yaw(self):
  geometry=Geometry();vertices=tip_vertices(geometry)
  part=next(p for p in layout()[0] if p['name']=='south_top')
  # Last measured poses from the short-pad-approach simulator trace.
  north=np.array([.2509857416,.0004260623,.8389458060])
  north_rotation=quat_xyzw_to_matrix([-.0004704312,-.0000587979,-.0486574993,.9988153577])
  origin=north-north_rotation@np.array([0., WIDTH/2, HEIGHT/2])
  flap=np.array([.2300900817,-.1813154370,.8841057420])
  flap_rotation=quat_xyzw_to_matrix([-.0343610197,-.7065882087,-.7059490681,-.0344702341])
  inward=flap_rotation[:,2]
  def depths(target):
   tcp,wrist=target
   return [max(float(((mesh@wrist.T+tcp-flap)@inward).max())
      for _,link,mesh in geometry.local_hand((.012,.012)) if link==pad)+part['size'][2]/2
      for pad in ('tip_left','tip_right')]
  previous=depths(wall_target(origin,north_rotation,'right',HEIGHT,WIDTH,vertices,0.,level_right_pad=True))
  corrected=depths(folded_flap_target(flap,flap_rotation,part['size'],vertices,0.))
  self.assertGreater(abs(previous[0]-previous[1]),.002)
  self.assertLess(abs(corrected[0]-corrected[1]),.00005)
  self.assertLess(max(map(abs,corrected)),.00005)

 def test_parked_props_clear_full_inverted_carton_footprint(self):
  r=np.diag([1.,-1.,-1.]);corners=np.array([[x,y,z]for x in(-1,1)for y in(-1,1)for z in(-1,1)])
  raw=[]
  for p in layout()[0]:
   q=p['quat'];Q=quat_xyzw_to_matrix([*q[1:],q[0]]);raw.extend(np.asarray(p['pos'])+(corners*np.array(p['size'])/2)@Q.T)
  v=np.array(raw)@r.T
  # Same table-supported rolling prediction as the reviewed reach screen.
  y=CENTER[1]+WIDTH/2+.003+HEIGHT+(WIDTH/2+.0045)
  o=np.array([CENTER[0],y,TOP+.0002-v[:,2].min()]);v+=o
  for name,(p,Q,size)in parked_cutter().items():
   prop=np.asarray(p)+(corners*np.array(size)/2)@Q.T
   separation=np.maximum(prop.min(0)-v.max(0),v.min(0)-prop.max(0))
   self.assertGreaterEqual(float(separation.max()),.01,name+' lies in landing footprint')
  self.assertEqual(CUTTER_PAD_START[:2],CUTTER_START[:2])
if __name__=='__main__':unittest.main()
