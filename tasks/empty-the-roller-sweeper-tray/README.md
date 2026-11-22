# Empty the roller sweeper's debris tray

Pull the loaded drawer clear of its guides, empty the dry spill into the waste bin, and slide the tray home.

## Setup and reset

Park the compact sweeper with its 134 × 62 mm drawer seated under the cover. Load six rice grains, six split lentils and six oat flakes. Place a 250 × 224 mm receiving bin alongside the drawer opening. Recover all 18 pieces and spread them across the seated tray in three rows.

## Scoring

Pull the pan at least 48 mm forward while level with the guides, lift it over 35 mm, then tip it over 45 degrees. Score settled pieces in the receiver divided by 18, capped at 0.75. Full credit requires all 18 pieces resting in the receiver for 0.5 s. Both rear corners of the drawer must finish within 2 mm of the back stop, with the floor supported by the chassis and both grippers clear for 0.4 s. The pose check also limits displacement to 10 mm and rotation to 10 degrees.

## Reproduce

Use Python 3.12 with FFmpeg, libx264, EGL and DejaVu Sans. Run from this folder. Use default variant 0 for the submitted-layout simulation. The action limit is 21 simulated seconds.

```sh
python -m pip install -r requirements.txt
export MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1
python run.py sweeper --out runs/scripted --video
python run.py sweeper --out runs/replay --replay runs/scripted/commands.json
python run.py sweeper --out runs/idle --control
python run.py sweeper --out runs/partial --detail-control
python run.py sweeper --out runs/short-return --short-return
```

In the configured Dream runtime, install the program's pinned adapter and run the same task with the same action limit. The evaluator saves three-camera policy results, actions, score trace and replay video. The transcript is written locally and should be inspected before sharing.

```sh
python -m pip install --no-deps 'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'
python evaluate.py sweeper --out runs/model --video
```

## Fidelity

The loaded drawer slides under a cover and two fixed retaining lips before lifting and tipping. Eighteen free rigid food pieces model six rice grains, six split lentils and six oat flakes. Each piece moves by contact and gravity. The 0.5 ms physics step resolves thin flakes. Fine dust adhesion, the motor and powered sweeping are outside the task. Guide spacing and release forces have not been measured on a commercial unit.

## Physical check

Use a switched-off floor sweeper with a removable slide-out tray, a waste bin, dry uncooked rice, split lentils and oat flakes. Six of each are large enough to count in a close-up. Start with all 18 in the tray, and push the tray fully home. Film the runners and the disposal bin. The robot must pull the tray out, tip the contents into the bin without spilling, slide the tray back along its guides and release it.

Count each type of debris in the receiver, tray and spill area. The three counts must add back to six for each type. To judge reseating, align witness marks on the tray and housing, confirm both guides overlap the tray and check it stays home after release. The digital fixture has a 134 × 62 mm pan, a 64 mm withdrawal stroke and a fixed chassis. Its geometry and release force have not been measured on a production sweeper.

The Gtech SW02 manual (page 6) provides the real task analogue, including grasping and pulling the tray before tipping it over a bin. The simulated drawer uses a rigid grip and guides. Fine dust adhesion, powered sweeping and the deforming window of the commercial model are outside the task.
https://www.gtech.co.uk/media/productfileupload/5/6/5633_sw02_manual_update_aw.pdf
