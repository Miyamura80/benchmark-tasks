# Frame and tools

All dimensions are millimetres. The fixture frame has +X inboard, +Y toward the rear axle and +Z up. Coordinates below use the front sprocket centre as origin unless another frame is specified.

| Part | Submitted geometry |
| --- | --- |
| Drivetrain | 28-tooth front ring and 16-tooth fixed rear sprocket. Centre distance 405.68. Pitch radii 56.71 and 32.55. Parallel shaft axes along X. Align tooth centre planes to within 0.5. Use a closed 86-pitch, 12.70-pitch single-speed chain with 7.70 rollers and 3.20 clear inner width. |
| Rear assembly | Independent shaft, 10 diameter, 134 long. Rear guides are 1.6 thick at X = ±6 relative to the rear tooth plane, with 37.05 outer radius. Target 1 mN·m rear drag (roughly 3.1 g hanging at the rear pitch radius). |
| Frame | Bottom-bracket centre (45,0,0), shell OD 41, width 68. Seat cluster (45,80,320), head cluster (45,−300,295). The rear dropouts are at (−17,405.68,0) and (107,405.68,0). Tube ODs are 28 seat, 36 down, 26 top, 16 chainstay and 12 seatstay. The chainstays start at (19,10,−4) and (71,10,−4). Use straight tubes and a fixed service stand. |
| Crank | Service crank radius 58, counterweighted. Pedal spindle 16 diameter × 112 long, parallel to the shafts. Its exposed end accepts the socket along X. The compact crank adapts the task to a tabletop YAM workspace. |
| Arm mounting | In the simulator world the front tooth centre is (−8,−48,271). Left base centre is (−290,130,290), right base (−286,−180,175). Both arms are on the drive side. Left base quaternion is (1,0,0,0) in MuJoCo WXYZ order. Use the right base orientation from the supplied station. Bolt the two bases to separate rigid pedestals at those poses. |

## Offset jaws

Use one independent finger extension on each stock left gripper finger. The supplied shoe STLs seat against the outside of the corresponding fingertip. Their 4.2 mm bores accept the 4 mm rods. Print the shoes, dry-fit on spare fingertips, then bond the mating cavities and rod roots using a structural epoxy suitable for the actual finger material. Remove both shoes with the spare fingers when reverting the gripper. The design is untested on a physical YAM.

Each jaw has three rigid 4 mm OD segments. A thin-wall tube with joined sections and a bonded rubber-faced pad is a proposed build. Set the bends in a jig using the coordinates below. Use the closed **left TCP** as the datum. The sign is +1 for the stock left tip and −1 for the stock right tip.

| Point | X | Y | Z |
| --- | ---: | ---: | ---: |
| Rod root | 0 | 9 × sign | −6 |
| First bend | −48 | 94 + 9 × sign | 12 |
| Last bend | −85 | 108 + 6.3 × sign | 57 |
| Pad centre | −85 | 108 + 2.3 × sign | 75 |

Each pad is 15 × 3.6 × 6, faces parallel to the TCP XZ plane. The simulated gap is 1.0 at the zero slide setting. Check that the actual jaws clear one another throughout their travel and grip both side plates with the rod roots fixed. The simulator treats each extension as rigid with 5.1 g added per moving finger, and the pad coefficient of sliding friction is 1.0. Adhesive fit, pad friction, tube deflection and grasp force need physical checks.

## Right pedal socket

The supplied pedal-socket STL uses the right TCP as datum. It has a 19 × 19 square bore, four 4 mm walls and 14 mm insertion depth. Bore centre is (0,0,40), with bore axis along TCP X. The integral 8 × 12 × 22 stem is centred at (0,0,17). Secure this stem in a rigid gripper-body bracket with its bore aligned to the spindle. The robot slips the socket over the spindle, turns the crank by contact, then withdraws axially. The socket is fixed to the gripper body in simulation. A physical bracket must be matched to the particular coupler and fastener pattern.

## Rebuild the printable parts

The two shoe STLs and socket STL are already included. Dimensions are in mm. To regenerate the shoes, export the installed stock fingertip meshes and render the OpenSCAD solid differences.

```sh
cd hardware
python export_fingers.py
openscad -o shoe-left.stl -D mode=1 tools.scad
openscad -o shoe-right.stl -D mode=-1 tools.scad
openscad -o pedal-socket.stl -D mode=0 tools.scad
```

OpenSCAD reported all three as simple solids. The stock-finger export uses the native zero-slide pose and the asset mesh transform. The 0.20 mm allowance follows the outer normal side of each shoe.
