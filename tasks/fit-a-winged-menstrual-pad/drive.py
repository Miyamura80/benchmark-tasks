"""Execute joint targets on the native robot at 20 Hz; checkpoint each stage."""
from pathlib import Path
import json
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from robot import Arm, DT, HZ, SIDES


class Driver:
    def __init__(self, model, materials, output, seed=0):
        self.model = model
        self.data = mj.MjData(model)
        self.arms = {s: Arm(model, s) for s in SIDES}
        self.materials = materials
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.commands = []
        self.fixture_commands = []
        self.states = []
        self.readings = []
        self.stage = 'reset'
        self.stages = []
        self.last = {}
        self.ik_error = {}
        self.seed = seed
        self.resume_stages={}
        self.resume_root=None

    def reset(self, poses):
        for side, (position, rotation, opening) in poses.items():
            arm = self.arms[side]; joints, err = arm.solve(position, rotation)
            if err > .004:
                raise ValueError(f'{side} initial pose unreachable at {err:.4f} m weighted residual')
            arm.reset(self.data, joints, opening)
            self.last[side] = np.r_[joints, opening]
        mj.mj_forward(self.model, self.data)
        self.materials.last_judged_time=float(self.data.time)
        self.states.append(self.data.qpos.copy())
        self.readings.append(self.materials.read(self.data))

    def action(self, command, fixture=None, guard=None):
        if fixture is None:
            fixture = [float(self.data.actuator('frame_height').ctrl[0]),float(self.data.actuator('frame_angle').ctrl[0]),self.materials.inspection_phase]
        lift,angle,phase=fixture
        self.data.actuator('frame_height').ctrl[0]=lift
        self.data.actuator('frame_angle').ctrl[0]=angle
        self.materials.inspection_phase=int(phase);self.materials.inspection=phase>0
        self.fixture_commands.append([float(lift),float(angle),int(phase)])
        a = np.asarray(command)
        if a.shape != (2, 7) or not np.isfinite(a).all():
            raise ValueError('Expected two finite joint/opening position targets')
        for side, row in zip(SIDES, a):
            self.arms[side].command(self.data, row[:6], row[-1]); self.last[side] = row.copy()
        prior_time = self.data.time
        violation=None
        for _ in range(round(1/(HZ*DT))):
            mj.mj_step(self.model, self.data)
            self.materials.update(self.data, DT)
            if guard is not None and violation is None:
                violation=guard(self.data)
        mj.mj_forward(self.model, self.data)
        self.states.append(self.data.qpos.copy())
        self.commands.append(a.copy())
        self.materials.advance_judge(self.data)
        self.readings.append(self.materials.read(self.data))
        if violation is not None:
            raise RuntimeError(violation)
        unstable = any(self.data.warning[k].number for k in (int(mj.mjtWarning.mjWARN_BADQPOS), int(mj.mjtWarning.mjWARN_BADQVEL), int(mj.mjtWarning.mjWARN_BADQACC)))
        if unstable or abs(self.data.time - prior_time - 1/HZ) > 1e-8 or np.max(abs(self.data.qvel)) > 1000 or not np.isfinite(self.data.qpos).all():
            raise RuntimeError('Native dynamics became unstable')

    def reuse_prefix(self, root, through):
        self.resume_root=Path(root)
        rows=[json.loads(x) for x in (self.resume_root/'stages.jsonl').read_text().splitlines()]
        for row in rows:
            self.resume_stages[(row['stage'],row['begin_action'])]=row
            if row['stage']==through:return
        raise ValueError('Requested restart stage is missing')

    def move(self, stage, goals, duration, strict=True, joint_travel=False, joint_targets=None, guard=None):
        self.stage = stage; start_index = len(self.commands)
        old=self.resume_stages.get((stage,start_index))
        if old:
            end=old['end_action'];snap=np.load(self.resume_root/'checkpoints'/f'{end:04d}.npz',allow_pickle=False)
            if 'integration' in snap:
                self.model.eq_data[:]=snap['eq_data'];mj.mj_setState(self.model,self.data,snap['integration'],mj.mjtState.mjSTATE_INTEGRATION);mj.mj_forward(self.model,self.data)
                info=json.loads(str(snap['material']));self.materials.ready=set(info['ready']);self.materials.events=info['events'];self.materials.n=info['n'];self.materials.last_judged_time=float(self.data.time)
                
                for key,value in info.get('judge',{}).items():setattr(self.materials,key,value)
                adh=self.materials.adh;adh.events=info['adh_events'];adh.held={int(k):v for k,v in info['held'].items()};adh.strong={int(k):v for k,v in info['strong'].items()};adh.overload={int(k):v for k,v in info['overload'].items()}
                self.states.extend(snap['qpos'][1:]);self.commands.extend(snap['commands']);self.fixture_commands.extend(snap['fixture']);self.readings.extend(json.loads(str(snap['readings'])));self.last={side:row.copy() for side,row in zip(SIDES,self.commands[-1])}
            else:
                for command,fixture in zip(snap['commands'],snap['fixture'],strict=True):self.action(command,fixture)
            self.stages.append(old)
            import shutil
            dst=self.output/'checkpoints';dst.mkdir(exist_ok=True);shutil.copyfile(self.resume_root/'checkpoints'/f'{end:04d}.npz',dst/f'{end:04d}.npz')
            with (self.output/'stages.jsonl').open('a') as stream:stream.write(json.dumps(old)+'\n')
            print('Restored',stage,end,flush=True);return
        starts = {s:(*self.arms[s].pose(self.data), self.last[s][-1]) for s in goals}
        deltas = {s:Rotation.from_matrix(goals[s][1] @ starts[s][1].T).as_rotvec() for s in goals}
        count = max(1, round(duration*HZ)); residual = {s:0. for s in goals}
        transfer = {}
        if joint_travel or joint_targets is not None:
            for side,(p,R,gap) in goals.items():
                arm=self.arms[side]
                q0=self.data.qpos[arm.positions].copy()
                if joint_targets is None:
                    q1,err=arm.solve(p,R)
                else:
                    q1=np.asarray(joint_targets[side],dtype=float)
                    if q1.shape!=(6,) or not np.isfinite(q1).all() or np.any(q1<arm.low) or np.any(q1>arm.high):
                        raise ValueError('Invalid free-motion joint waypoint')
                    err=0.
                transfer[side]=(q0,q1)
                residual[side]=float(err)
        for j in range(count):
            t = (j+1)/count; w = t*t*(3-2*t)
            for s,(position, rotation, opening) in goals.items():
                p0, R0, g0 = starts[s]
                if s in transfer:
                    q0,q1=transfer[s]
                    self.last[s]=np.r_[(1-w)*q0+w*q1,g0+w*(opening-g0)]
                    continue
                R = Rotation.from_rotvec(w*deltas[s]).as_matrix() @ R0
                q, err = self.arms[s].solve((1-w)*p0+w*np.asarray(position),R,self.last[s][:6])
                self.last[s] = np.r_[q, g0+w*(opening-g0)];residual[s]=max(residual[s],float(err))
            self.action([self.last[s] for s in SIDES],guard=guard)
        end_index = len(self.commands)
        summary = {'stage':stage,'begin_action':start_index,'end_action':end_index,
                   'max_ik_residual_m':residual, 'state':self.readings[-1]}
        self.stages.append(summary)
        with (self.output/'stages.jsonl').open('a') as stream:
            stream.write(json.dumps(summary)+'\n')
        checkpoints=self.output/'checkpoints';checkpoints.mkdir(exist_ok=True)
        flag=mj.mjtState.mjSTATE_INTEGRATION;integ=np.empty(mj.mj_stateSize(self.model,flag));mj.mj_getState(self.model,self.data,integ,flag);adh=self.materials.adh
        material=dict(judge={k:getattr(self.materials,k) for k in ['inspection','inspection_phase','hold_s','proof_passed','proof_contacts','last_judged_time']},ready=list(self.materials.ready),events=self.materials.events,n=self.materials.n,adh_events=adh.events,held=adh.held,strong=adh.strong,overload=adh.overload)
        np.savez_compressed(checkpoints/f'{end_index:04d}.npz',qpos=np.array(self.states[start_index:end_index+1]),commands=np.array(self.commands[start_index:end_index]),fixture=np.array(self.fixture_commands[start_index:end_index]),integration=integ,eq_data=self.model.eq_data,material=json.dumps(material),readings=json.dumps(self.readings[start_index+1:end_index+1]))
        last=self.readings[-1];print(stage, end_index, last['score'],last['papers_clear'],last['wrapped'],last['core_mm'],flush=True)
        if strict and any(x>.016 for x in residual.values()):
            raise RuntimeError(f'IK failed during {stage}: {residual}')

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.output/'native.npz', qpos=np.array(self.states),
                            resumed_prefix=bool(self.resume_root), commands=np.array(self.commands), fixture=np.array(self.fixture_commands), dt=DT, hz=HZ)
        (self.output/'trace.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.readings))
