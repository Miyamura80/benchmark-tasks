"""Exact common manipulation budget and fixed terminal examiner protocol."""
import numpy as np

def audit(path):
    with np.load(path,allow_pickle=False) as rec:
        a=rec['commands'];f=rec['fixture'];states=rec['qpos']
        earned=np.flatnonzero(f[:,2]!=0)
        if not len(earned):
            raise ValueError('No examiner operation')
        start=int(earned[0]);rows=[]
        initial=np.array([0.,0.])
        for count,h,q,phase in [(20,.025,0,1),(50,.025,np.pi,2),(50,.025,np.pi,3),(50,.025,0,4),(14,.025,0,4)]:
            end=np.array([h,q])
            for k in range(1,count+1):
                t=k/count;w=t*t*(3-2*t)
                rows.append([*((1-w)*initial+w*end),phase])
            initial=end
        expected=np.array(rows)
        checks=dict(complete_states=(len(states)==len(a)+1),complete_targets=(a.shape==(len(f),2,7)),finite_controls=bool(np.isfinite(a).all() and np.isfinite(f).all()),common_budget=0<start<=1600,standardized_fixture_motion=(f[start:].shape==expected.shape and np.allclose(f[start:],expected,atol=1e-12,rtol=0)),fixture_quiescent_during_policy=bool(not np.any(f[:start])),fixed_arm_targets=bool(np.array_equal(a[start:],np.broadcast_to(a[start-1],a[start:].shape))))
        return dict(agent_controls=start,examiner_controls=len(a)-start,checks={k:bool(v) for k,v in checks.items()},passed=all(checks.values()))
