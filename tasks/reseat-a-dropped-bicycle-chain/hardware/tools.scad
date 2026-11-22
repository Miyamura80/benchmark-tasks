// Millimetres in the closed gripper TCP frame. Mode 1 and -1 select a mounting shoe.
// Export the reference fingers first with export_fingers.py.
$fn = 36;
mode = 1;
function root(s) = [0,9*s,-6];
function elbow(s) = [-48,94+9*s,12];
module capsule(a,b,r) {
    hull() {translate(a) sphere(r); translate(b) sphere(r);}
}
module shoe(s) {
    p = root(s);
    q = elbow(s);
    d = (q-p)/norm(q-p);
    difference() {
        union() {
            translate([-7,min(2*s,12*s),-12]) cube([14,10,13]);
            capsule(p,p+7*d,3.5);
        }
        translate([0,s*.20,0])
            if (s == 1) import("stock-left.stl"); else import("stock-right.stl");
        capsule(p-d,p+12*d,2.1);
    }
}
module socket() {
    difference() {
        translate([-7,-13.5,26.5]) cube([14,27,27]);
        translate([-8,-9.5,30.5]) cube([16,19,19]);
    }
    translate([-4,-6,6]) cube([8,12,22]);
}
if (mode == 0) socket(); else shoe(mode);
