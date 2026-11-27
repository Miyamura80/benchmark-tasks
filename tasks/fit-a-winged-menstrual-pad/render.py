"""Render actual MuJoCo states with simultaneous station and material views."""
import argparse
import json
from pathlib import Path
from PIL import Image
import mujoco as mj
import numpy as np
from footage import record,viewpoint
from run_provenance import reset_config,verify_binding,sha
from scene import GUSSET_Y,GUSSET_Z,PAD_Y,PAD_Z,build


def cameras():
    return {
       'wide':viewpoint((0,.025,.185),.98,116,-40),
       'paper':viewpoint((0,PAD_Y,PAD_Z),.36,95,-56),
       'over':viewpoint((0,GUSSET_Y,GUSSET_Z-.015),.40,90,-78),
       'under':viewpoint((0,GUSSET_Y,GUSSET_Z-.030),.42,222,19),
       'test':viewpoint((0,GUSSET_Y,GUSSET_Z-.025),.58,25,-12),
    }


def choose(k,row,stage,data,hold_index):
    """Select a close view of physical geometry in the current native state."""
    stage=stage.lower()
    center=data.geom('base_glue3').xpos
    # The garment is the scene's only flex. Its center vertex marks the gusset.
    gusset=data.flexvert_xpos[len(data.flexvert_xpos)//2]
    fitted=row.get('base_centered',False) and all(row.get('wrapped',{}).get(side,False) for side in ('near','far'))
    if 'hold fully inverted' in stage:
        if hold_index<17:
            return viewpoint(gusset,.306,90,-6),'Inverted backing profile' if fitted else 'Inverted garment profile'
        return viewpoint(gusset,.208,90,-89.9),'Both folded wings at full inversion' if fitted else 'Gusset underside at full inversion'
    if any(word in stage for word in ('turn the briefs','lift garment','raised','return the briefs','inspect','judging')):
        if 'inspect the released fit' in stage:
            return viewpoint(gusset,.338,90,-83),'Returned upright fit' if fitted else 'Returned upright garment'
        if row['frame_angle_deg']>150:
            return viewpoint(gusset,.228,90,-82),'Released wings during rotation' if fitted else 'Garment during rotation'
        return viewpoint(gusset,.385,38,-17),'Released garment inspection'
    wing_stage='liner' not in stage and 'paper' not in stage
    if ('near' in stage and wing_stage) or 'distal fingertip' in stage:
        return viewpoint(data.geom('near_glue').xpos,.148,341,67),'Near wing beneath gusset'
    if 'far' in stage and wing_stage:
        return viewpoint(data.geom('far_glue').xpos,.148,36,64),'Far wing beneath gusset'
    if 'holding jaws' in stage or 'holding hand' in stage or 'gusset and pad' in stage:
        return viewpoint(gusset,.218,0,70),'Wing contact and release'
    if 'turn absorbent' in stage or 'carry above' in stage or 'lower pad' in stage or 'lift both ends' in stage or 'clear and square' in stage:
        return viewpoint(center,.365,110,-65),'Pad transfer and orientation'
    if k<30 or any(word in stage for word in ('liner','paper','nose','loading','second grip')):
        return 'paper','Pad and release papers'
    if 'policy action' in stage or 'observation' in stage or 'replayed robot' in stage:
        return viewpoint(center,.36,95,-56),'Pad in the recorded state'
    return viewpoint(gusset,.338,90,-78),'Inside of the briefs'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--speed',type=int,default=1,help='Sample every Nth control at20fps;1 is native time')
    parser.add_argument('--still-action',type=int)
    parser.add_argument('--checkpoint',type=Path)
    parser.add_argument('--offset-mm',type=float,help='Must match recorded reset')
    parser.add_argument('--allow-unbound-diagnostic',action='store_true')
    args=parser.parse_args()
    config=reset_config(args.run,args.allow_unbound_diagnostic,args.offset_mm)
    m,_=build(float(config.get('offset_mm',0.)));d=mj.MjData(m);views=cameras()
    if args.still_action is None:
        record(m,d,args.run,args.out,'Fit a winged period pad into briefs',views,choose,
               speed=args.speed,allow_unbound_diagnostic=args.allow_unbound_diagnostic)
        return
    binding=verify_binding(args.run,m,args.allow_unbound_diagnostic)
    args.out.mkdir(parents=True,exist_ok=True)
    path=args.checkpoint if args.checkpoint else args.run/'native.npz'
    state=np.load(path,allow_pickle=False)['qpos'][args.still_action]
    if state.shape!=(m.nq,):raise ValueError('Still/model dimensions differ')
    d.qpos[:]=state;mj.mj_forward(m,d)
    with mj.Renderer(m,360,480) as renderer:
        canvas=Image.new('RGB',(960,720),'white')
        for index,name in enumerate(('wide','paper','over','under')):
            renderer.update_scene(d,camera=views[name]);canvas.paste(Image.fromarray(renderer.render()),((index%2)*480,(index//2)*360))
        canvas.save(args.out/'review.png')
    (args.out/'still-provenance.json').write_text(json.dumps(dict(source_binding=binding,
        state_file_sha256=sha(path),state_index=args.still_action,
        custom_checkpoint_diagnostic=args.checkpoint is not None),indent=2)+'\n')
if __name__=='__main__':main()
