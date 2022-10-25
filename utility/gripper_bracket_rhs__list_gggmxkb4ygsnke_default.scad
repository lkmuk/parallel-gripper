% scale(1000) import("gripper_bracket_rhs__list_gggmxkb4ygsnke_default.stl");

hinge_width = 20;
translate([19.5,-34,17]){
    cube([30,3,hinge_width],center=true);
    translate([-4.2,-6,0]){
        cylinder(r=5, h=hinge_width,center=true);
    }
}