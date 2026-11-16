// Flat 8:1 link lever, dimensions in mm. Laser-cut or machine from steel.
// Both links are on the same side of the pivot for same-direction travel.
// Printed copies are layout templates. Use a steel part for loaded trials.

width = 16;
thickness = 6;
pivot_diameter = 8.1;
link_diameter = 4.2;
carriage_radius = 10;
grip_radius = 80;
end_radius = width / 2;
$fn = 96;

linear_extrude(thickness)
    difference() {
        hull() {
            translate([-2, 0]) circle(r = end_radius);
            translate([grip_radius + 2, 0]) circle(r = end_radius);
        }

        circle(d = pivot_diameter);
        translate([carriage_radius, 0]) circle(d = link_diameter);
        translate([grip_radius, 0]) circle(d = link_diameter);
    }
