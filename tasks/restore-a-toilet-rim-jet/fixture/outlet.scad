// Rim-jet test fixture dimensions in millimetres.
// This removable saddle mounts a straight bulkhead outlet at 25° inward
// from vertical. A separate reservoir and equal-length tubes supply the jets.
clearance_diameter = 12.5;
tilt = 25;
$fn = 64;

difference() {
    union() {
        translate([-38,-16,6]) cube([24,32,6]);
        translate([-6,0,9]) rotate([0,tilt,0]) cube([28,30,6],center=true);
        hull() {
            translate([-16,-15,6]) cube([3,30,6]);
            translate([-18,0,9]) rotate([0,tilt,0]) cube([3,30,6],center=true);
        }
    }
    translate([-6,0,9]) rotate([0,tilt,0])
        cylinder(d=clearance_diameter,h=22,center=true);
    for (y = [-10,10])
        translate([-26,y,-1]) cylinder(d=4.5,h=20);
}
