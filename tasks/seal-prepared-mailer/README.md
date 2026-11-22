# Seal a preloaded peel-and-seal carton

Close the carton, press the full length of its factory-applied strip and withdraw. Two boxed products start inside. The release liner has already been removed.

## Run the task

Use Python 3.12 and the pinned YAM station checkout. The simulator needs a supported RTX GPU for its three cameras, including when PhysX runs on the CPU.

```sh
git clone https://github.com/kairosiann/yam-isaac-lab-arena
cd yam-isaac-lab-arena
git checkout 0059d02a1a91bc6834c1d0846022891df0c78730
./setup.sh
source ~/IsaacLab-Arena/yam.env
```

From this task directory, with the Dream credential configured on the evaluation host:

```sh
python native_run.py --device cpu --out scripted-trial
bash run_dream.sh dream-trial
```

Both commands require a new output directory. Both trials use seed 0, CPU PhysX at 240 Hz, the YAM action interface at 20 Hz, and a fixed carton layout. The scripted driver uses privileged contact feedback for inverse kinematics. Dream receives the standard overhead and two wrist camera views and measured joint positions. The scripted run ends on success. The model limit is 1,500 actions, equivalent to 75 simulated seconds. Hosting queue time is separate.

## Score

Each of the five adhesive cells forms a breakable joint after at least 100 ms of measured fingertip and flange contact. Minimum normal force is 0.55 N at the finger and 0.35 N at the flange, with a gap under 1.7 mm. A gap over 2.5 mm or a broken physical joint invalidates that bond. A grip that merely holds down the lid earns zero.

For a full score, all five bonds must survive, the absolute lid angle must be at most 3°, both products must be inside, carton speed must remain below 10 mm/s and both hands must withdraw for two continuous seconds. Each hand's tool centre must be more than 65 mm above the top or more than 90 mm beyond the carton side. Combined observed pad load over 30 N invalidates the episode. The environment computes these values from live contacts and poses and exposes them through the standard YAM task scorer.

The video of the scripted run includes the initial camera frame and all 418 actions. All five bonds survived the two-second hold. See `evidence/scripted.json` for the measured bond events and terminal score. The board holds the submitted videos and setup image.

## Negative controls

```sh
python native_run.py --control no-action --out neutral
python native_run.py --control one-missing --out missing
python native_run.py --control no-adhesive --out inert
python -m unittest test_hold
```

The first control leaves the carton open. The second skips the final adhesive cell. The third disables adhesion while keeping robot contact enabled. Each should score zero.

## Model trial

Dream-1 beta completed 1,500 actions at 20 Hz with 31 inference requests and scored 0.0. The model-run video shows the synchronized overhead, left wrist and right wrist cameras from that episode in order. See `evidence/dream.json` for the evaluation result. The scripted trial and model trial use the same scene, release criteria and scoring code.
