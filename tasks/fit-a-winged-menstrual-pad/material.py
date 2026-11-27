"""Load-driven progressive release, local pressure adhesion and a geometric ruler/photo judge."""
import mujoco as mj
import numpy as np
from contact_adhesive import TextileAdhesive
from scene import DX, DY, GUSSET_Y


def distances(points, vertices):
    return np.linalg.norm(points[:,None]-vertices[None,:],axis=-1).min(axis=1)


class Materials:
    def __init__(self, model, spec, no_tack=False, no_wing_tack=False):
        self.model=model; self.spec=spec;self.no_tack=no_tack;self.no_wing_tack=no_wing_tack
        self.adh=TextileAdhesive(model,spec['patches'],spec['moving'],'briefs')
        self.release_ids=np.array([model.equality(n).id for n,k in spec['bonds']]);self.release_keys=[k for n,k in spec['bonds']]
        self.a=model.eq_obj1id[self.release_ids];self.b=model.eq_obj2id[self.release_ids]
        self.a_local=model.eq_data[self.release_ids,:3].copy();self.b_local=model.eq_data[self.release_ids,3:6].copy()
        self.events=[];self.ready=set();self.n=0;self.peak={};self.inspection=False;self.hold_s=0.;self.proof_passed=False;self.proof_contacts=0;self.last_judged_time=None;self.inspection_phase=0
        self.paper={k:[model.body(n).id for n in nodes] for k,nodes in spec['liners'].items()}
        self.cores=np.array([model.body(f'core{i}').id for i in range(7)])
        self.patch_ids={g:model.geom(g).id for g in spec['patches']}
        self.all_gripper_geoms={g for g in range(model.ngeom) if model.geom(g).name.startswith(('left_tip_','right_tip_'))}
        self.robot_geom=np.array(['_contact_' in model.geom(i).name and model.geom(i).name.startswith(('left_','right_')) for i in range(model.ngeom)])
        def proof_object(geom_id):
            body=int(model.geom_bodyid[geom_id])
            while body:
                name=model.body(body).name
                if name.startswith(('core','near_','far_','main_','inspection_frame','bearing_carriage')):
                    return True
                body=int(model.body_parentid[body])
            return False
        self.material_geom=np.array([proof_object(i) for i in range(model.ngeom)])

    def update(self,d,dt):
        if self.inspection and d.ncon:
            g=d.contact.geom; valid=(d.contact.dist<=0)&(d.contact.efc_address>=0)
            robot=(g>=0)&self.robot_geom[np.maximum(0,g)]
            material=((g>=0)&self.material_geom[np.maximum(0,g)])|(d.contact.flex==self.adh.flex)
            if np.any(valid&((robot[:,0]&material[:,1])|(robot[:,1]&material[:,0]))):self.proof_contacts+=1
        self.n+=1
        if self.n%2: return
        dt=dt*2
        active=d.eq_active[self.release_ids].astype(bool)
        if active.any():
            select=d.efc_type==int(mj.mjtConstraint.mjCNSTR_EQUALITY);ids=d.efc_id[select];forces=d.efc_force[select]
            # Native point constraints have three world-coordinate reaction rows.
            where=np.flatnonzero(np.r_[True,ids[1:]!=ids[:-1]]) if len(ids) else np.empty(0,int)
            eq_first={int(ids[r]):int(r) for r in where}
            Ra=d.xmat[self.a].reshape(-1,3,3);Rb=d.xmat[self.b].reshape(-1,3,3)
            a=d.xpos[self.a]+np.einsum('nij,nj->ni',Ra,self.a_local);b=d.xpos[self.b]+np.einsum('nij,nj->ni',Rb,self.b_local)
            normal=Rb[:,:,2];opening=np.einsum('ij,ij->i',a-b,normal)
            for i in np.where(active & (opening>.000020))[0]:
                eid=self.release_ids[i];r=eq_first.get(int(eid));
                if r is None: continue
                tension=-forces[r:r+3]@normal[i]
                if tension>.110:
                    d.eq_active[eid]=False;key=self.release_keys[i];self.events.append({'time':round(d.time,4),'event':'release_zone','paper':key,'zone':self.model.equality(eid).name,'tension_n':round(float(tension),4)})
            for key in self.paper:
                indexes=[idx for idx,k in zip(self.release_ids,self.release_keys) if k==key]
                if not d.eq_active[indexes].any():self.ready.add(key)
        exposed={g for g,key in self.spec['groups'].items() if key in self.ready and not self.no_tack and (not self.no_wing_tack or key=='main')}
        if exposed:self.adh.update(d,exposed,dt,pressure=.025,dwell=.055,limit=.45)

    def advance_judge(self, d):
        """Advance proof once after a physical action; observation is read-only."""
        now=float(d.time)
        if self.last_judged_time is None:
            self.last_judged_time=now
            return
        elapsed=now-self.last_judged_time
        if elapsed<=0:
            return
        if elapsed>.0500001:
            raise ValueError('Retention proof must be sampled at every 20 Hz action')
        state=self.read(d)
        retained=(self.inspection and state['frame_lift_mm']>=20.
                  and state['frame_angle_deg']>165.
                  and self.proof_contacts==0 and state['base_centered']
                  and all(state['wrapped'].values()) and all(state['papers_clear'].values()))
        self.hold_s=self.hold_s+elapsed if retained else 0.
        self.last_judged_time=now
        if self.hold_s>=1.95:
            self.proof_passed=True

    def jaw_loads(self,d):
        summary={};load=np.zeros(6)
        for idx,c in enumerate(d.contact):
            if c.efc_address<0 or min(c.geom)<0:continue
            a,b=map(int,c.geom)
            if a in self.all_gripper_geoms:tool,other=a,b
            elif b in self.all_gripper_geoms:tool,other=b,a
            else:continue
            if not self.material_geom[other]:continue
            mj.mj_contactForce(self.model,d,idx,load)
            if load[0]<.003:continue
            jaw=self.model.geom(tool).name.split('_contact_')[0];name=self.model.geom(other).name
            by_item=summary.setdefault(jaw,{})
            by_item[name]=round(by_item.get(name,0.)+float(load[0]),4)
        return summary

    def core_xyz(self,d):
        return d.xpos[self.cores]+DX/2*d.xmat[self.cores].reshape(-1,3,3)[:,:,0]

    def textile(self,vertices,xy):
        # Interpolate the currently deformed triangle beneath this marking.
        # Input points are judged from the calibrated overhead and side views.
        faces=self.spec['faces'];tri=vertices[faces];p=tri[:,0,:2];a=tri[:,1,:2]-p;b=tri[:,2,:2]-p
        det=a[:,0]*b[:,1]-a[:,1]*b[:,0];v=xy-p
        with np.errstate(divide='ignore',invalid='ignore'):
            u=(v[:,0]*b[:,1]-v[:,1]*b[:,0])/det;w=(a[:,0]*v[:,1]-a[:,1]*v[:,0])/det
        candidates=np.where((u>=-.0001)&(w>=-.0001)&(u+w<=1.0001))[0]
        if not len(candidates):return None
        # The gusset is the inner panel in this fixture; edge bands sit at the waist.
        idx=candidates[0];T=tri[idx];n=np.cross(T[1]-T[0],T[2]-T[0]);n/=max(np.linalg.norm(n),1e-12)
        if n[2]<0:n=-n
        return float(T[0,2]+u[idx]*(T[1,2]-T[0,2])+w[idx]*(T[2,2]-T[0,2])),n

    def read(self,d):
        frame=d.body('inspection_frame');basis=frame.xmat.reshape(3,3);origin=frame.xpos
        def local(points): return (points-origin)@basis
        vertices=local(d.flexvert_xpos);cores=local(self.core_xyz(d));rows=self.spec['rows'];cols=self.spec['cols'];centeridx=(rows//2)*cols+cols//2;mid=vertices[centeridx];rot=np.einsum('ij,njk->nik',basis.T,d.xmat[self.cores].reshape(-1,3,3))
        papers={};protected={}
        for key,nodes in self.paper.items():
            sample=np.array([d.geom(f'{key}_leaf{i}').xpos.copy() for i in range(len(nodes)-1)])
            targets=np.array([d.geom(g).xpos.copy() for g,k in self.spec['groups'].items() if k==key]);dist=distances(sample,targets)
            papers[key]=bool(dist.min()>.018);protected[key]=int(sum(d.eq_active[i] for i,k in zip(self.release_ids,self.release_keys) if k==key))
        stable=[]
        for i in (1,3,5):
            p=local(d.geom(f'base_glue{i}').xpos);s=self.textile(vertices,p[:2]);stable.append(s is not None and -.002<=p[2]-s[0]<=.010 and rot[i,:,2]@s[1]<-.65)
        center_error=cores[3,:2]-mid[:2];yaw=np.degrees(np.arctan2(abs(cores[5,1]-cores[1,1]),abs(cores[5,0]-cores[1,0])+1e-12))
        fit=bool(papers['main'] and np.all(stable) and abs(center_error[0])<=.017 and abs(center_error[1])<=.009 and yaw<=12.)
        wing={};wing_gaps={};edge_positions=vertices[[centeridx-cols//2,centeridx+cols//2],1];lower,upper=sorted(edge_positions)
        for key in ['near','far']:
            g=self.patch_ids[f'{key}_glue'];p=local(d.geom_xpos[g]);ori=basis.T@d.geom_xmat[g].reshape(3,3);root=local(d.body(f'{key}_wing0').xpos);sgn=1 if root[1]>mid[1] else -1
            checks=[];gaps=[]
            for off in (-.008,0.,.008):
                sample=p+ori[:,0]*off;surface=self.textile(vertices,sample[:2]);inset=upper-sample[1] if sgn==1 else sample[1]-lower
                if surface is None:checks.append(False);continue
                gap=sample[2]-surface[0];gaps.append(gap);checks.append(.006<=inset<=.055 and -.012<=gap<=.003 and ori[:,2]@surface[1]>.50)
            wing[key]=bool(papers[key] and all(checks));wing_gaps[key]=round(1000*float(np.mean(gaps)),2) if gaps else None
        clean=all(papers.values());count=sum(wing.values());frame_height=float(d.joint('inspection_lift').qpos[0]);angle=float(d.joint('inspection_turn').qpos[0]);
        score=(.20 if clean else .10 if papers['main'] else 0.)
        if fit: score=max(score,.50+.20*count)
        if self.proof_passed and self.proof_contacts==0 and self.inspection_phase==4 and abs(angle)<.06 and fit and count==2 and clean:score=1.
        return dict(score=round(score,2),loaded_jaws_n=self.jaw_loads(d),end_spacing_mm=round(float(1000*np.linalg.norm(cores[0]-cores[6])),1),papers_clear=papers,base_centered=fit,wrapped=wing,center_error_mm=np.round(center_error*1000,1).tolist(),yaw_deg=round(float(yaw),1),wing_gap_mm=wing_gaps,frame_lift_mm=round(1000*frame_height,1),frame_angle_deg=round(float(np.degrees(angle)),1),inverted_hold_pass=self.proof_passed,inspection_contact_substeps=self.proof_contacts,released_hold_s=round(self.hold_s,2),liner_bonds=protected,textile_bonds=[self.adh.patches[g] for g in self.adh.held],core_mm=np.round(cores[3]*1000,1).tolist(),gusset_mm=np.round(mid*1000,1).tolist())
