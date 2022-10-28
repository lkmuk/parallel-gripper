% scale(1000) import("rod_outer.stl");

// Append pure shapes (cube, cylinder and sphere), e.g:
// cube([10, 10, 10], center=true);
// cylinder(r=10, h=10, center=true);
// sphere(10);

rod_thickness = 5;
bar_width = 10;
translate([-20-bar_width/2, 21, 0   ]){
        cube([bar_width, 43, rod_thickness]);
}
