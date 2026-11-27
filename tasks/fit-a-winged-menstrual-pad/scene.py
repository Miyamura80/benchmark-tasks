"""Fresh, face-down winged pad, separate release papers and intact suspended briefs."""
from xml.etree import ElementTree as ET
import numpy as np
import mujoco as mj
from dream_examples.yam_mujoco import yam_env
from robot import stock_station, xml
from contact_adhesive import declare

PAD_Y = -.165
PAD_Z = .224
GUSSET_Y = .190
GUSSET_Z = .260
DX = .0328
DY = .01465
NCORE = 7
TAB_ANGLE = .85
TEXTILE = (.32, .34, .54, 1)
TOP = (.96, .95, .93, 1)
SEAM = (.63, .73, .77, 1)
PAPER = (.72, .54, .31, 1)
GLUE = (.88, .80, .64, 1)
WOOD = (.26, .20, .16, 1)
METAL = (.32, .36, .40, 1)

def box(body, name, pos, size, rgba, **kwargs):
    return xml(body,'geom',name=name,type='box',pos=pos,size=size,rgba=rgba,**kwargs)

def rail(body,name,a,b,r,color):
    return xml(body,'geom',name=name,type='capsule',fromto=(*a,*b),size=r,rgba=color)

def garment(world, bench, carriage):
    # A continuous surface with three boundary loops: waist and two legs.
    # The strip printed over the middle eight centimetres is the cotton gusset.
    xs = np.array([-.173,-.150,-.126,-.105,-.081,-.054,-.027,0,.027,.054,.081,.105,.126,.150,.173])
    widths = np.interp(abs(xs),[0,.055,.085,.105,.126,.150,.173],[.034,.034,.040,.050,.067,.091,.114])
    ys = np.linspace(-1,1,7); verts=[];faces=[];pins=[]
    for i,(x,w) in enumerate(zip(xs,widths)):
        for u in ys:
            verts.append([x,GUSSET_Y+w*u,GUSSET_Z+.008*(abs(x)/.173)**2+.004*u*u*(abs(x)/.173)**2])
    for i in range(len(xs)-1):
        for j in range(6):
            a=i*7+j;b=a+7;faces.extend(((a,b,b+1),(a,b+1,a+1)))
    for sign,j in [(-1,0),(1,6)]:
        strand=[(j,7+j)]
        for x in np.linspace(-.125,.125,7):
            outer=len(verts);w=.125+.024*(1-(x/.15)**2)
            rise=-.094*(1-(x/.15)**2)
            verts.extend(([x,GUSSET_Y+sign*w,GUSSET_Z+.010+rise],[x,GUSSET_Y+sign*(w-.022),GUSSET_Z-.002+rise]))
            strand.append((outer,outer+1))
            if abs(x)<.003:pins.append(outer)
        strand.append(((len(xs)-1)*7+j,(len(xs)-2)*7+j))
        for (a,b),(c,d) in zip(strand,strand[1:]):
            tri=[(a,c,d),(a,d,b)];faces.extend(tri if sign==1 else [tuple(reversed(t)) for t in tri])
    pins.extend([0,6,(len(xs)-1)*7,len(xs)*7-1])
    flex=xml(world,'flexcomp',name='briefs',dim=2,type='direct',point=verts,element=faces,mass=.026,radius=.00070,rgba=TEXTILE)
    xml(flex,'pin',id=pins);xml(flex,'edge',equality='true',damping=.004,solref='.006 1')
    xml(flex,'contact',selfcollide='none',internal='false',solref='.003 1',solimp='.98 .999 .0002',priority=2,friction='.75 .003 .0001')
    # All six cloth pins correspond to small, visible waistband clips. The
    # textile between them is free, including the entire gusset and pad area.
    # The metal perimeter lies outside the textile. Short clamp jaws reach
    # inward to the pinned edge without preloading the rest of the waistband.
    def upright(point,sign):
        x,y,z=point
        return (x,y+sign*.018,z-.011)
    def clip(point,sign,name):
        x,y,z=point;out=upright(point,sign)
        rail(world,name+'_post',out,(x,out[1],z-.003),.0025,METAL)
        box(world,name,(x,y+sign*.009,z-.0029),(.006,.011,.0016),METAL)
    for sign in (-1,1):
        side=verts[pins[0 if sign==-1 else 1]]
        for end in (-1,1):
            corner=verts[(0 if end==-1 else len(xs)-1)*7+(0 if sign==-1 else 6)]
            rail(world,f'outer_rim_{sign}_{end}',upright(corner,sign),upright(side,sign),.0025,METAL)
            clip(corner,sign,f'corner_clip_{sign}_{end}')
        clip(side,sign,f'side_clip_{sign}')
    # A dropped cross-bar connects the two rim halves well below either end
    # grip. A single cantilever axle leaves both longitudinal approaches clear.
    for sign in (-1,1):
        xx=.183; yy=GUSSET_Y+sign*.114
        rail(world,f'crossmember_end_{sign}',(.173,yy+sign*.018,GUSSET_Z-.003),(xx,yy,GUSSET_Z-.080),.0025,METAL)
    rail(world,'crossmember',(.183,GUSSET_Y-.114,GUSSET_Z-.080),(.183,GUSSET_Y+.114,GUSSET_Z-.080),.0025,METAL)
    # A rear-only slatted cotton support transfers smoothing pressure to the
    # cantilever at the opposite end of the frame. The middle and both wing
    # folds remain accessible from below. Under inversion the support is above
    # the cloth, separated from the pad by the cotton.
    for i, x in enumerate((.074,.085,.096)):
        rail(world,f'rear_rest_slat_{i}',(x,GUSSET_Y-.023,.248),(x,GUSSET_Y+.023,.248),.004,METAL)
    rail(world,'rear_rest_spine',(.074,GUSSET_Y,.241),(.100,GUSSET_Y,.241),.004,METAL)
    rail(world,'rear_rest_bracket',(.100,GUSSET_Y,.241),(.183,GUSSET_Y,GUSSET_Z-.080),.004,METAL)
    yf=GUSSET_Y+.146; zf=GUSSET_Z-.025
    rail(world,'axle_bracket',upright(verts[pins[1]],1),(0,yf,zf),.003,METAL)
    rail(world,'inspection_axle',(0,yf,zf),(0,GUSSET_Y+.198,zf),.004,METAL)
    rail(bench,'bearing_pedestal',(-.014,GUSSET_Y+.210,.008),(-.014,GUSSET_Y+.210,zf+.024),.005,METAL)
    box(carriage,'bearing_slide',(0,GUSSET_Y+.210,zf-.014),(.006,.011,.020),METAL)
    box(carriage,'bearing_saddle',(0,GUSSET_Y+.194,zf-.008),(.007,.007,.003),METAL)
    box(bench,'bearing_foot',(0,GUSSET_Y+.205,.005),(.030,.020,.005),METAL)
    return np.array(verts),np.array(faces),[i for i in range(len(verts)) if i not in pins],len(xs),len(ys)

def pad(world,asset,eq,x_shift=0.):
    cores=[];patches={};groups={};liners={};bonds=[]
    widths=np.array([.018,.030,.033,.032,.032,.033,.030,.018]); start=-NCORE*DX/2+x_shift
    for i in range(NCORE):
        b=xml(world if i==0 else b,'body',name=f'core{i}',pos=(start,PAD_Y,PAD_Z) if i==0 else (DX,0,0))
        if i==0:xml(b,'freejoint',name='pad_free')
        else:
            for key,axis,k in [('pitch',(0,1,0),.035),('twist',(1,0,0),.025)]:
                xml(b,'joint',name=f'{key}{i}',type='hinge',axis=axis,stiffness=k,damping=.00017,armature=.0000008)
        # Convex tapered pads with rounded transverse cross-sections. The last
        # 16 mm at either nose taper smoothly to a trim seam in plan view.
        pts=[]
        for t in np.linspace(0,1,5):
            w=(1-t)*widths[i]+t*widths[i+1];xx=DX*t
            if i==0 and t<.5:w*=np.sqrt(max(.035,1-(1-2*t)**2))
            if i==NCORE-1 and t>.5:w*=np.sqrt(max(.035,1-(2*t-1)**2))
            for u,zz in [(-1,0),(-.93,-.00135),(-.65,-.0022),(.65,-.0022),(.93,-.00135),(1,0),(.92,.00165),(-.92,.00165)]:pts.append((xx,u*w,zz))
        mesh=f'pad_profile{i}';xml(asset,'mesh',name=mesh,vertex=pts);xml(b,'geom',name=f'absorbent{i}',type='mesh',mesh=mesh,mass=.00093,rgba=TOP,solref='.003 1',friction='.85 .003 .0001')
        # Embossed channels are cosmetic and rotate with the physical core.
        for sign in (-1,1):
            xml(b,'geom',name=f'channel_{i}_{sign}',type='capsule',fromto=(.001,sign*.55*widths[i],-.00222,DX-.001,sign*.55*widths[i+1],-.00222),size=.00030,rgba=SEAM,contype=0,conaffinity=0,mass=0)
        cores.append(b)
        if 1<=i<=5:
            g=f'base_glue{i}';box(b,g,(DX/2,0,.00176),(DX*.36,.012,.00014),GLUE,mass=.000013,solref='.003 1');patches[g]=f'core{i}';groups[g]='main'
    for sign in (-1,1):
        key='near' if sign==-1 else 'far';parent=cores[3]
        for i in range(3):
            b=xml(parent,'body',name=f'{key}_wing{i}',pos=(DX/2,sign*.032,0) if i==0 else (0,sign*DY,0));parent=b
            xml(b,'joint',name=f'{key}_bend{i}',type='hinge',axis=(1,0,0),stiffness=.0002,damping=.000025,armature=.0000001)
            width=[.019,.025,.023][i]
            box(b,f'{key}_sheet{i}',(0,sign*DY/2,0),(width,DY/2-.00006,.00034),TOP,mass=.00018,solref='.003 1')
            if i==2:
                g=f'{key}_glue';box(b,g,(0,sign*DY/2,.00045),(.015,DY/2-.002,.00013),GLUE,mass=.00002,solref='.003 1');patches[g]=f'{key}_wing2';groups[g]=key
    def paper(key, x, y, z, dx, halfwidth, anchors):
        n=len(anchors); bodies=[]
        for i in range(n+1):
            length=dx if i<n else .016
            b=xml(world if i==0 else b,'body',name=f'{key}_paper{i}',pos=(x,y,z) if i==0 else (dx,0,0),euler=(0,-TAB_ANGLE if i==n else 0,0))
            if i==0:xml(b,'freejoint',name=f'{key}_paper_free')
            else:xml(b,'joint',name=f'{key}_paper_hinge{i}',axis=(0,1,0),stiffness=.00005 if i==n else .000012,damping=.000002,armature=.000000015)
            box(b,f'{key}_leaf{i}',(length/2,0,0),(length/2-.00007,halfwidth,.0001),(.78,.57,.28,1) if i==n else PAPER,mass=.00008 if key=='main' else .000025,friction='.6 .0001 .00001',solref='.003 1')
            bodies.append(b.get('name'))
            if i<n:
                name=f'{key}_release{i}';xml(eq,'connect',name=name,body1=b.get('name'),body2=anchors[i],anchor=(dx/2,0,0),solref='.004 1',solimp='.96 .99 .0001');bonds.append((name,key))
        liners[key]=bodies
    paper('main',start+DX,PAD_Y,PAD_Z+.00204,DX,.0145,[f'core{i}' for i in range(1,6)])
    for sign,key in [(-1,'near'),(1,'far')]:
        paper(key,x_shift-.0165,PAD_Y+sign*(.032+2.5*DY),PAD_Z+.00068,.011,.006,[f'{key}_wing2']*3)
    return patches,groups,liners,bonds

def build(offset_mm=0.):
    root,world=stock_station();world.find("body[@name='left_base']").set('pos','-.33 .28 .025')
    xml(world,'geom',name='table',type='box',size=(.54,.45,.02),pos=(0,.025,-.021),rgba=(.49,.49,.46,1))
    for side,x,y in [('left',-.33,.28),('right',.33,-.14)]:box(world,side+'_mount',(x,y,.010),(.052,.052,.012),METAL)
    box(world,'paper_tray',(0,PAD_Y,PAD_Z-.0102),(.075,.082,.008),WOOD)
    for x in (-.052,.052):
        for y in (-.059,.059):rail(world,f'trayleg_{x}_{y}',(x,PAD_Y+y,.003),(x,PAD_Y+y,PAD_Z-.020),.005,WOOD)
    box(world,'paper_disposal_tray',(.035,-.340,.010),(.103,.048,.010),(.46,.60,.57,1))
    yam_env.camera(world,'top',(.03,-.10,.98),(0,0,.21))
    carriage=xml(world,'body',name='bearing_carriage',gravcomp=1);xml(carriage,'joint',name='inspection_lift',type='slide',axis=(0,0,1),range=(-.001,.028),damping=12,armature=.12);frame=xml(carriage,'body',name='inspection_frame',gravcomp=1);xml(frame,'joint',name='inspection_turn',type='hinge',pos=(0,GUSSET_Y,GUSSET_Z-.025),axis=(0,1,0),range=(-.012,3.16),damping=.4,armature=.018);xml(root.find('actuator'),'position',name='frame_angle',joint='inspection_turn',kp=90,kv=3,ctrlrange=(0,np.pi),forcerange=(-9,9));xml(frame,'inertial',pos=(0,GUSSET_Y,.15),mass=.7,diaginertia=(.016,.013,.019));xml(root.find('actuator'),'position',name='frame_height',joint='inspection_lift',kp=4800,kv=70,ctrlrange=(0,.025),forcerange=(-80,80));vs,faces,moving,nrow,ncol=garment(frame,world,carriage);eq=xml(root,'equality');patches,groups,liners,bonds=pad(world,root.find('asset'),eq,offset_mm*.001)
    declare(eq,patches,moving,'briefs')
    model=mj.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    return model,dict(verts=vs,faces=faces,moving=moving,rows=nrow,cols=ncol,patches=patches,groups=groups,liners=liners,bonds=bonds)
