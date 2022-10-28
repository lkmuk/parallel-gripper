% scale(1000) import("rod_inner_lhs_top.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

rod_thickness = 5;
bar_width = 10;
translate([0, 15, 19]){
    cylinder(r=13, h=rod_thickness);
    translate([-bar_width/2, 0, 0]){
        cube([bar_width, 39, rod_thickness]);
    }
}