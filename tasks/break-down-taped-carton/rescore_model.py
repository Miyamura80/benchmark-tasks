"""Independently recompute every model-trial score from logged native samples."""
import argparse
import hashlib
import json
from pathlib import Path
from scoring import BreakdownScorer


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('trial',type=Path)
    args=parser.parse_args()
    trial=args.trial
    result=json.loads((trial/'result.json').read_text())
    tracker=BreakdownScorer(require_turnover=True)
    count=0
    for line in (trial/'trajectory.jsonl').open():
        row=json.loads(line)
        measured=tracker.update(row['score_sample'])
        if measured!=row['metrics']:
            raise RuntimeError('Scorer replay mismatch at native control '+str(count+1))
        count+=1
    if not count or tracker.value!=result['task_score']:
        raise RuntimeError('Missing model trajectory or mismatched final score')
    transcript=json.loads((trial/'dream-transcript.json').read_text())
    if not transcript['requests']:
        raise RuntimeError('No genuine Dream inference requests were recorded')
    if transcript['executed_control_steps']!=count:
        raise RuntimeError('Model actions and executed native controls differ')
    video=trial/'model-run.mp4'
    summary=dict(model='Dream-1',checkpoint=transcript['model'],score=tracker.value,
                 native_control_steps=count,simulated_seconds=count/20.,
                 max_control_steps=12000,independently_matched_score_samples=count,
                 metrics=tracker.result(),model_requests=len(transcript['requests']),
                 model_failure=transcript['failure'],
                 elapsed_wall_seconds=transcript['elapsed_wall_seconds'],
                 video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
                 trajectory_sha256=hashlib.file_digest((trial/'trajectory.jsonl').open('rb'),'sha256').hexdigest(),
                 native_runner_sha256=hashlib.sha256(Path(__file__).with_name('run.py').read_bytes()).hexdigest())
    (trial/'model-result.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
