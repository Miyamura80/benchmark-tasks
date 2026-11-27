"""Bind run evidence to its exact initial model, source and reset configuration."""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco as mj
SOURCE=Path(__file__).resolve().parent
MEDIA_ONLY={'footage.py','render.py'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():return {p.name:sha(p) for p in sorted(SOURCE.glob('*.py'))}


def model_signature(model):
    buffer=np.empty(mj.mj_sizeModel(model),dtype=np.uint8)
    mj.mj_saveModel(model,buffer=buffer)
    return hashlib.sha256(buffer.tobytes()).hexdigest()


def capture_run_source(output,model,config):
    """Call immediately after build, before material mutation or native actions."""
    path=Path(output);path.mkdir(parents=True,exist_ok=True)
    if any(path.iterdir()):raise FileExistsError('Refusing to overwrite or rebind an occupied output folder')
    value=dict(config=config,source_sha256=source_hashes(),initial_model_sha256=model_signature(model),
               nq=model.nq,nv=model.nv,capture='Before native actions; no credentials serialized')
    with (path/'run-source.json').open('x') as f:json.dump(value,f,indent=2);f.write('\n')
    return value


def reset_config(run,allow_unbound=False,offset=None):
    p=Path(run)/'run-source.json'
    if not p.exists():
        if not allow_unbound:raise ValueError('Missing run-source.json; use a fresh bound run or explicit diagnostic mode')
        return dict(offset_mm=0. if offset is None else offset)
    config=json.loads(p.read_text())['config']
    if offset is not None and abs(offset-float(config.get('offset_mm',0)))>1e-12:
        raise ValueError('Offset differs from recorded reset')
    return config


def verify_binding(run,model,allow_unbound=False):
    p=Path(run)/'run-source.json';issues=[];record=None
    if not p.exists():issues.append('Missing frozen source and reset binding')
    else:
        record=json.loads(p.read_text())
        if model_signature(model)!=record['initial_model_sha256']:issues.append('Compiled model differs from recorded reset')
        actual=source_hashes()
        for name,value in record['source_sha256'].items():
            if name not in MEDIA_ONLY and actual.get(name)!=value:issues.append('Runtime source differs: '+name)
        if record['nq']!=model.nq or record['nv']!=model.nv:issues.append('Model dimensions differ')
    if issues and not allow_unbound:raise ValueError('; '.join(issues))
    return dict(verified=not issues,issues=issues,binding=record,rendering_source_sha256=source_hashes())


def seal_run_evidence(output,completed,failure_type=None):
    output=Path(output)
    files=('native.npz','trace.jsonl','stages.jsonl','result.json','run-source.json')
    missing=[name for name in files if not (output/name).is_file()]
    record=dict(completed=bool(completed),failure_type=failure_type,missing=missing,
                input_sha256={name:sha(output/name) for name in files if (output/name).is_file()})
    (output/'run-completion.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def verify_completion(run):
    run=Path(run);p=run/'run-completion.json'
    if not p.exists():return dict(verified=False,completed=False,issues=['Missing completion seal'])
    record=json.loads(p.read_text());issues=[]
    expected_files={'native.npz','trace.jsonl','stages.jsonl','result.json','run-source.json'}
    recorded=record.get('input_sha256',{})
    if not isinstance(recorded,dict) or set(recorded)!=expected_files:issues.append('Completion seal must bind exactly all five raw files')
    if not isinstance(recorded,dict):recorded={}
    for name in expected_files:
        expected=recorded.get(name)
        if expected is None or not (run/name).is_file() or sha(run/name)!=expected:issues.append('Evidence digest differs: '+name)
    if record.get('missing'):issues.append('Completion seal has missing evidence')
    return dict(verified=not issues,completed=bool(record.get('completed')),issues=issues,seal=record)
