# Fit a winged period pad into briefs

Peel the release papers, center the pad on the inside of the gusset with its absorbent face up, and stick both wings to the underside. Release the garment before inspection.

## Physical setup and reset

Use an unused regular winged disposable pad measuring approximately 230 mm long and 152 mm across the open wings. The narrow body is 64–66 mm wide. The simulated pad has a nonwoven cover, an absorbent core, a flexible backing and pressure-sensitive adhesive on the backing and both wings. The UK Care and Protect regular ultra pad, supplier SKU 108-0037, is listed at 230 × 152 mm. Its current release-paper arrangement and peel force have not been verified. [Product dimensions](https://www.janitorialdirect.co.uk/care-and-protect-regular-ultra-pads-with-wings-x500.html)

Prepare the three papers before a trial. Select a pad with a separate main liner and a wing liner that spans the folded wings. With the protective papers still bonded, lift the unattached bridge of the shared wing liner away from both wings and snip **only the free paper bridge** with scissors. Open the wings without peeling their individual paper covers. Keep the main liner attached. Orient the pad with all three covered adhesive regions up. Fold a free 16 mm paper margin on each liner upward about 49° to make an accessible tab. Use a compatible lot with enough free margin. For the modeled preparation, the central sheet covers a 164 × 29 mm rectangle and each wing sheet covers 33 × 12 mm, in addition to the free tab. Use the actual adhesive boundaries to locate the split. Record the lot and photograph the preparation from both sides. This three-piece preparation is part of the benchmark. Commercial instructions describe placing the pad inside the underwear and wrapping its wings underneath. [Manufacturer guidance](https://www.always.com/en-us/period-hub/first-period-and-puberty/how-to-use-a-pad)

Use one intact, dry pair of cotton-rich bikini briefs. The inside faces up. The midpoint between the front and back waist centers lies at the center of the gusset. Choose briefs measuring 68 mm across the gusset there when opened on the frame. Mark the gusset center and the longitudinal centerline lightly with washable textile pencil. Record make, size, cloth width and clip distances and keep them fixed across trials. Use identical clean briefs after each trial and a new prepared pad. Discard used pads and papers.

Use millimeters for the fixture coordinates below. The x = 0 plane crosses the middle of the loading shelf and the gusset. The y = 0 plane is 190 mm toward the loading shelf from the gusset center. Positive x points toward the right arm, positive y toward the briefs, and positive z up. The tabletop is at z = −1 mm in the simulation. The vertex positions, six cloth pins, clips and fabric triangles are specified in scene.py. The open textile has one waist opening and two leg openings.

| Object | Construction and placement |
|---|---|
| Station | Two six-joint YAM arms with stock opposed fingers. Mount centers at (−330, 280, 25) and (330, −140, 25) mm, facing inward along x. Tabletop measures 1080 × 900 mm. |
| Loading shelf | A 150 × 164 × 16 mm wooden shelf on four legs. Set the pad on its absorbent face with long axis along x, center over the shelf and both ends overhanging. Pad center starts at (0, −165, 224) mm. |
| Paper tray | 206 × 96 mm, centered at (35, −340) mm. Start empty. |
| Garment | The modeled main panel is 346 mm long, 228 mm wide at its ends and 68 mm at its center. Nominal gusset center is (0, 190, 260) mm. The six clips hold only the outside waistband. The gusset and leg openings remain unpinned. Three rear support slats sit below the cotton. |
| Inspection frame | Four corner clips centered near x = ±173 mm and y = 190 ± 114 mm, plus side clips near x = 0 and y = 190 ± 149 mm. The side clips hold the loose waistband 84 mm below nominal gusset height and leave the leg openings accessible from above. Use rigid perimeter rods outside the cloth and a far-side bearing carried on a vertical slide. Keep the table and loading shelf clear of the frame's full rotation. |
| Rear cotton support | Three fixed transverse slats, one longitudinal spine and one diagonal brace rotate with the garment frame. Use smooth 8 mm diameter metal rod for all five pieces. Mount at the coordinates below. |

### Build and check the rear support

Each coordinate pair below gives the centers of the two rounded ends of an 8 mm diameter rod. The straight section runs between those centers. The exposed tip extends another 4 mm at each end.

| Piece | First end (x, y, z), mm | Second end (x, y, z), mm |
|---|---|---|
| Slat 1 | (74, 167, 248) | (74, 213, 248) |
| Slat 2 | (85, 167, 248) | (85, 213, 248) |
| Slat 3 | (96, 167, 248) | (96, 213, 248) |
| Longitudinal spine | (74, 190, 241) | (100, 190, 241) |
| Diagonal brace | (100, 190, 241) | (183, 190, 180) |

Fasten the slats rigidly to the spine and the brace to the center of the right crossmember. The three crosswise rods are 54 mm tip to tip, spaced 11 mm center to center with 3 mm clear between them. Check that their tops share a plane at z = 252 mm. With the fixture upright, the slats lie 8 mm below the nominal center plane of the gusset. The slat footprint is x = 70–100 mm and y = 163–217 mm. The brace lies below that contact surface.

Clamp the four waistband corners and two side-waist points, then let the cotton settle under gravity. The rear panel can bear on the slats when the hands press the pad into it. It remains free to slide or lift away. The garment center, front panel, leg hems and both wing-folding regions stay unpinned and clear of the rear support. Keep the pad on the loading shelf until the trial starts. The slats, spine and brace stay fitted through lift, inversion and return. The six waistband clips provide the only cloth attachments to the frame.

Before attaching the fresh pad, photograph its adhesive patches and the paper tabs beside a ruler. Use that photograph to locate the same measurement points on the opposite face. Use overhead and underside cameras for the centerline and wing inset, and a side camera for gaps. Place the scale in the garment plane. A paper outline with the pad center and three transverse marks per wing can assist comparison.

## Physical judge

Assign the highest qualifying tier in the final upright pose. For 1.00, also check the entire inspection sequence. Measure in coordinates fixed to the garment frame. The original inside and outside of the briefs retain those labels throughout rotation. Use the marked cotton surface as zero for every base and wing gap, including the aft station above the slats. During inspection, count robot contact with any moving frame part, including a slat or brace, as a hands-clear failure.

| Score | Observation |
|---|---|
| 0.00 | The main paper is still on the pad, and none of the conditions below hold. |
| 0.10 | The main paper is clear. Mark the centers of its five covered 32.8 mm sections and the centers of the five matching adhesive patches. Every paper mark must be more than 18 mm from every adhesive mark on the body. For each wing, apply this distance to the three covered 11 mm paper sections and the center of that wing’s adhesive patch. Free tabs do not count. |
| 0.20 | All three papers are clear. |
| 0.50 | The main paper is clear. Pad center is within 17 mm longitudinally and 9 mm transversely of the gusset center. The long axis differs by at most 12°. The adhesive face points toward the garment at three stations on the base (approximately 66 mm forward of center, center, and 66 mm aft). Each lies within 10 mm above the cloth; simulation allows 2 mm of contact tolerance below its midsurface. The face normal lies within 49.5° of the opposing cloth normal. |
| 0.70 / 0.90 | The 0.50 fit passes and one / both uncovered wings wrap under their respective gusset edges. On each wing adhesive patch use three spots spaced along x at −8, 0, +8 mm from its center. Every spot lies 6–55 mm inward of the gusset edge and within 12 mm below its underside. The adhesive points toward the cloth, within 60° of its inward-facing direction. Simulation allows 3 mm of contact tolerance above its midsurface. |
| 1.00 | Both wrapped wings and the centered pad survive the hands-clear inspection. Lift the fixture 25 mm, turn 180°, hold fully inverted for 2.5 s and return it upright. Both robot arms must clear the pad, briefs and moving fixture throughout the lift, turn, hold and return. The observed fit must persist beyond 165° inversion with at least 20 mm lift for 1.95 consecutive seconds, and pass again within 3.4° of upright. |

Record synchronized overhead, underside and side views at 20 Hz. Include the 25 mm lift scale and a frame angle mark. Use the side view to read the pad-to-cotton gap at each marked longitudinal station, and an underside view to measure all three wing insets. Check both arms and the full moving frame across the entire inspection. Score from the images, ruler and angle markings. Mechanical adhesion loads and hidden solver constraints are diagnostics. For the simulation equivalent, material.py samples attached geometry in the garment frame at 20 Hz. It checks robot contact at every 1 ms physics step throughout inspection. Repeated calls to the observer cannot advance the proof.

## Running the simulation

The reference run used Python 3.12.14, MuJoCo 3.3.7, the stock YAM assets from dreamlabs 1.0.23, NumPy 2.2.6 and SciPy 1.18.1. Pillow 13.0.0 and FFmpeg 7.1.5 were installed for image and video work. Run these commands from this directory. Use a new output directory for each run so its commands, source binding and completion record remain together. The reset and stock IK multistart use fixed seeds. Each action specifies six joint positions and one finger opening per arm at 20 Hz. The script and Dream-1 each receive up to 1,600 manipulation actions (80 seconds). The examiner then runs exactly 184 additional 20 Hz actions (9.2 seconds), repeating the last robot commands and moving only the fixture. Both controllers receive the same inspection schedule.

    python check_reset.py
    python run.py --mode script --out runs/demo
    python run.py --mode replay --replay runs/demo/native.npz --out runs/replay
    python run.py --mode replay --no-wing-tack --replay runs/demo/native.npz --out runs/no-wing-tack
    python run.py --mode replay --no-tack --replay runs/demo/native.npz --out runs/no-tack
    python run.py --mode idle --out runs/idle
    MUJOCO_GL=egl python run.py --mode dream --out runs/model
    MUJOCO_GL=egl python render.py runs/demo --out runs/demo-film --speed 1

The Dream adapter supplies top and wrist images plus joint positions and finger aperture. Configure the Dream SDK credentials before inference. Its policy action budget is 1,600, followed by the same examiner operation as the demonstration. Dry observations, restarts from engineering checkpoints and counterfactuals cannot establish a fresh demonstration or a model score.

## Fidelity and validation

The pad body has seven tapered panels. Three compliant panels represent each wing. The modeled briefs weigh 26 g. Five 23.6 × 24 mm base zones and two 30 × 10.7 mm wing zones can grip contacted textile triangles after at least 0.025 N of normal pressure for 55 ms. A joined triangle releases after its peak vertex load exceeds 0.45 N for 4 ms. Each paper liner has separate local attachments that part under more than 0.11 N of normal tension and 0.020 mm of opening. The material stiffnesses and pressure, dwell and separation thresholds are engineering assumptions. They have not been calibrated against a purchased pad or tested on hardware. The garment model preserves its openings and movable textile but omits knit detail, pile and fine crumpling. The inspection frame provides a repeatable substitute for a person lifting and turning the garment.

The native reset audit is in evidence/reset-audit.json. The scripted demonstration completed from the initial pad and garment in 1,525 manipulation actions and 184 inspection actions, scoring 1.00 in 85.45 simulated seconds. It took about 7 minutes 25 seconds of wall time on a shared CPU, including model construction and checkpoint writes. Final placement error was 13.8 mm longitudinally and 0.6 mm transversely. Both wing patches and all three scored base stations remained adhered through inversion and return. The fixed-angle hold lasted 2.50 s. The fit had qualified continuously for 2.95 s by the end of that hold (frame 1645), reaching 3.40 s at 165.5° during the return (frame 1654). The examiner recorded zero robot–assembly contacts.

Dream-1 scored 0.00 on a fresh reset of the same compiled model. The run completed all 1,784 controls in 89.20 simulated seconds. Its manipulation budget covered 32 initial observation-history controls followed by 1,568 policy controls from 196 model responses. The examiner then ran the same 184 inspection controls as in the demonstration. The model session ran continuously. All three release papers stayed attached and the covered pad remained on the loading shelf. Neither the backing nor either wing bonded to the briefs. There were zero robot contacts with the assembly during inspection. The model run used Python 3.12.16, DreamLabs 1.0.23 and dreamlabs-client 1.1.7, with the same MuJoCo, NumPy, SciPy and Pillow versions as the demonstration.

Independent action replay reproduced all 1,709 scripted controls from a fresh reset with bit-identical native coordinates and scoring records. A separate fresh replay of all 1,784 Dream controls reproduced every scoring record and material event. Cross-host Dream joint initialization differed by at most 1.84e-10 in native coordinates; the largest post-action coordinate difference was 1.17e-10 and the final difference was 1.74e-18. These Dream coordinates agree numerically, not bit for bit. See evidence/replay-audit.json.
