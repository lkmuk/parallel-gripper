% scale(1000) import("rod_inner_bottom.stl");


rod_thickness = 5;
bar_width = 10;
z_center = -15;

translate([0,-35,z_center]){
  cube([12, 28, rod_thickness], center=true);
}


mirror([0,1,0])translate([0, 15, z_center-rod_thickness/2]){
    cylinder(r=11, h=rod_thickness);
}