"""Synchronized rendering of recorded native states; no interpolation or animation."""
import json,subprocess
from pathlib import Path
import mujoco as mj
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from run_provenance import sha,verify_binding,verify_completion
from protocol import audit as audit_protocol


def viewpoint(lookat,distance,azimuth,elevation):
    camera=mj.MjvCamera();camera.type=mj.mjtCamera.mjCAMERA_FREE
    camera.lookat[:]=lookat
    camera.distance,camera.azimuth,camera.elevation=distance,azimuth,elevation
    return camera


def font(size):
    for name in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf','DejaVuSans.ttf'):
        try:return ImageFont.truetype(name,size)
        except OSError:pass
    return ImageFont.load_default()


def record(model,data,run,film,header,views,view_at,setup_frame=0,speed=1,allow_unbound_diagnostic=False):
    run,film=Path(run),Path(film)
    if not isinstance(speed,int) or speed<1:raise ValueError('Playback stride must be a positive integer')
    binding=verify_binding(run,model,allow_unbound_diagnostic)
    inputs=[run/n for n in ('native.npz','trace.jsonl','stages.jsonl','result.json','run-source.json','run-completion.json')]
    hashes={p.name:sha(p) for p in inputs if p.is_file()}
    info=json.loads((run/'result.json').read_text());mode=info['mode']
    with np.load(run/'native.npz',allow_pickle=False) as z:
        states=z['qpos'];commands=z['commands'].reshape(-1,2,7);fixture=z['fixture'].reshape(-1,3)
        resumed=bool(z['resumed_prefix']) if 'resumed_prefix' in z.files else False
    rows=[json.loads(x) for x in (run/'trace.jsonl').read_text().splitlines()]
    stages=[json.loads(x) for x in (run/'stages.jsonl').read_text().splitlines()]
    if states.shape!=(len(commands)+1,model.nq) or len(rows)!=len(states) or len(fixture)!=len(commands):
        raise ValueError('State, target, fixture, trace or model parity failed')
    if info.get('completed_actions')!=len(commands):raise ValueError('Result/target action count differs')
    if not np.isfinite(states).all() or not np.isfinite(commands).all() or not np.isfinite(fixture).all():
        raise ValueError('Nonfinite native evidence')
    if not 0<=setup_frame<len(states):raise ValueError('Setup frame outside native evidence')
    labels={'script':('demo.mp4','Scripted robot run'),'dream':('model-run.mp4','Dream-1 policy'),
            'replay':('replay.mp4','Command replay'),'idle':('idle.mp4','No-action control'),
            'dry-dream':('dry-observation.mp4','Observation check; no inference')}
    filename,kind=labels.get(mode,('run.mp4','Recorded native run'));status=[]
    if not binding['verified']:status.append('UNBOUND DIAGNOSTIC')
    completion=verify_completion(run)
    if not completion['verified'] or not completion['completed']:status.append('INCOMPLETE OR UNVERIFIED COMPLETION')
    if resumed:status.append('RESTORED DIAGNOSTIC')
    if info.get('ended_after') is not None or info.get('completed') is not True or info.get('failure_type') is not None:status.append('INCOMPLETE RUN')
    config=(binding['binding'] or {}).get('config',{})
    if config.get('no_tack') or config.get('no_wing_tack'):status.append('ADHESION CONTROL')
    if mode=='dry-dream':status.append('DRY; NO INFERENCE')
    examiner=np.flatnonzero(fixture[:,2]>0)
    try:protocol=audit_protocol(run/'native.npz')
    except (ValueError,IndexError,KeyError):protocol=dict(passed=False,checks={})
    full_examiner=bool(protocol['passed'])
    if mode in ('script','dream','replay') and not full_examiner:status.append('INCOMPLETE OR ALTERED EXAMINER')
    policy_rows=max(0,(int(examiner[0]) if len(examiner) else len(commands))-32) if mode=='dream' else 0
    reported=sum(int(c.get('executed',0)) for c in info.get('calls',[]))
    verified_model_rows=policy_rows if info.get('model_called') is True and reported==policy_rows else 0
    if mode=='dream' and not verified_model_rows:
        status.append('UNVERIFIED MODEL ATTEMPT');kind='No verified model controls'
    if status:filename='diagnostic.mp4';kind+=' | '+' | '.join(status)
    stage_map={k:s['stage'] for s in stages for k in range(s['begin_action']+1,s['end_action']+1)}
    hold_actions=np.flatnonzero(fixture[:,2]==3)
    first_hold_state=int(hold_actions[0])+1 if len(hold_actions) else len(states)
    def stage_at(k):
        if k in stage_map:return stage_map[k]
        if k==0:return 'Reset'
        phase=int(fixture[k-1,2])
        if phase:return {1:'Lift garment for inspection',2:'Turn the briefs upside down',
                         3:'Hold fully inverted',4:'Return and inspect the released fit'}.get(phase,'Garment inspection')
        if mode in ('dream','dry-dream'):
            if k<=32:return 'Observation history; no inference action'
            return 'Dream-1 policy action' if verified_model_rows else 'Unverified policy action'
        return {'idle':'No-action control','replay':'Replayed robot action'}.get(mode,'Recorded robot action')
    film.mkdir(parents=True,exist_ok=True)
    renderers=[mj.Renderer(model,height=h,width=w) for w,h in ((640,480),(480,480))]
    main_font,small_font=font(22),font(15)
    video=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1120x558','-r','20','-i','pipe:0','-c:v','libx264','-threads','2','-preset','fast','-crf','21',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(film/filename)],stdin=subprocess.PIPE)
    indices=sorted(set(range(0,len(states),speed))|{setup_frame,len(states)-1})
    try:
        for k in indices:
            data.qpos[:]=states[k];mj.mj_forward(model,data)
            reference,reference_label=view_at(k,rows[k],stage_at(k),data,k-first_hold_state)
            canvas=Image.new('RGB',(1120,558),(247,245,241));draw=ImageDraw.Draw(canvas)
            title=f'{header} | {kind} | Native state playback';title_font=main_font
            for size in (20,18,16,14):
                if draw.textlength(title,font=title_font)<=1094:break
                title_font=font(size)
            draw.text((13,9),title,fill=(31,34,37),font=title_font)
            for x,renderer,selection in ((0,renderers[0],'wide'),(640,renderers[1],reference)):
                camera=views[selection] if isinstance(selection,str) else selection
                renderer.update_scene(data,camera=camera);canvas.paste(Image.fromarray(renderer.render()),(x,41))
            draw.text((13,530),f'{k/20:05.1f} s native | {stage_at(k)}',fill=(36,41,43),font=small_font)
            draw.text((651,530),f'{reference_label} | Score {rows[k].get("score",0):.2f} | {speed}x playback',fill=(36,41,43),font=small_font)
            if k==setup_frame:canvas.save(film/'setup.png')
            video.stdin.write(canvas.tobytes())
    finally:
        try:video.stdin.close()
        finally:
            result=video.wait()
            for renderer in renderers:renderer.close()
    if result:raise RuntimeError('H.264 encoding failed')
    if hashes!={p.name:sha(p) for p in inputs if p.is_file()}:raise RuntimeError('Evidence changed during rendering')
    manifest=dict(mode=mode,status=status,video_file=filename,resumed_prefix=resumed,
        verified_model_control_rows=verified_model_rows,unverified_nonbootstrap_rows=policy_rows-verified_model_rows,
        input_sha256=hashes,source_binding=binding,completion_binding=completion,full_examiner=full_examiner,protocol_audit=protocol,native_control_hz=20,encoded_fps=20,
        nominal_playback_multiplier=speed,sampled_control_indices=indices,source_state_count=len(states),
        rendering='Playback of recorded native states without interpolation')
    (film/'playback.json').write_text(json.dumps(manifest,indent=2)+'\n')
