"""Run a blocked-lumen control on the same command sequence."""
import argparse, json
import numpy as np
from pathlib import Path
from run import Hose,Station
def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('commands',type=Path)
 ap.add_argument('output',type=Path)
 args=ap.parse_args()
 f=Hose(); t=Station(f)
 # A sleeve keeps the walls from reopening during this counterfactual.
 jid=t.model.joint('lumen_gap').id
 t.model.jnt_range[jid]=[.00015,.0015]
 for row in json.loads(args.commands.read_text()): t.action(np.array(row).reshape(2,7))
 r=t.reading(); assert r['score']==0 and abs(r['bend_deg'])<10 and r['flow_ml_s']<.2*205,r
 args.output.write_text(json.dumps(r,indent=2)+'\n')
if __name__=='__main__': main()
