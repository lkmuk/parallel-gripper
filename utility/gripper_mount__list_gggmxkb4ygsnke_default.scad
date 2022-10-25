% scale(1000) import("gripper_mount__list_gggmxkb4ygsnke_default.stl");


gripper_height = 40;
mount_width = 33*2;
width_stage2 = 28*2;
body_length_stage1 = 41;
body_length_stage2 = 8;

translate([-2, -mount_width/2, -3]){
  cube([body_length_stage1,mount_width,gripper_height]);
  translate([body_length_stage1, (mount_width-width_stage2)/2 ,0]){
      cube([body_length_stage2, width_stage2, gripper_height]);
  }
}