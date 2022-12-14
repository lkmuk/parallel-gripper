import xml.etree.ElementTree as ET 

from urdf_kit.automation.base import generic_xacro_export, export_target_spec
from urdf_kit.edit_joints import grab_expected_joints_handle
from urdf_kit.edit_links import rename_link
from urdf_kit.edit_transmission import make_simple_transmission_elem
from urdf_kit.graph.simplify import fix_revolute_joint
from urdf_kit.graph.tree import kinematic_tree
from urdf_kit.misc import remove_subelement_by_tag

from pathlib import Path
import sys
this_dir = Path(__file__).resolve().parent
sys.path.append(this_dir)
from gripper_model import gripper_actuation_concept

class frozen_gripper_urdf_output(export_target_spec):
    def __init__(self, target_full_path, reusable_macro_name, freeze_angle: float):
        """
        freeze_angle (float):
            in radian
            the angle will be applied based on the actuation_spec
            to all 6 URDF joints (the other two do not exist in the URDF).
        """
        super().__init__(target_full_path, reusable_macro_name)
        assert isinstance(freeze_angle, float)
        self.angle = freeze_angle

class gripper_xacro_export(generic_xacro_export):
    def __init__(
        self, 
        urdf_xacro_output_spec: export_target_spec,
        OnshapeToRobotProjectDir, # <--- should be a full path
        actuation_spec : gripper_actuation_concept,
        extra_urdf_output_frozen_angle: frozen_gripper_urdf_output = None
    ):
        """ parallel-link four bar gripper URDF

        At the moment, this export pipeline will only 
        handle the URDF xacro.
        and the gazebo xacro has to be authored manually.

        Special stuff that we will consider here:
            * mimic tags (xhs_j4 are left to the gazebo stuff!!!) 
            * transmission tags (for all active joints)
        """
        xacro_output_list = [urdf_xacro_output_spec]
        if extra_urdf_output_frozen_angle is not None:
            self.has_frozen_output = True
            xacro_output_list.append(extra_urdf_output_frozen_angle)
        else:
            self.has_frozen_output = False
        super().__init__(
            xacro_output_list = xacro_output_list,
            OnshapeToRobotProjectDir=OnshapeToRobotProjectDir
        )
        self.actuation_spec = actuation_spec
        print("")
        print("Gripper actuation specification:")
        self.actuation_spec.print()  

        self.mimic_spec_list = actuation_spec._passive_joint_list
        self.mimic_joint_name_list = [spec.to for spec in self.mimic_spec_list]

        # # better first run `_validate_consistency_with_src_URDF`
        # self.mimic_joint_handle_list = grab_expected_joints_handle(
        #     self.urdf_root, 
        #     self.mimic_joint_name_list
        # )


    def _gen_joint_names_ordered_1_2_3(self, which_side):
        assert which_side in ("lhs","rhs")
        return [which_side+f"_j{i}" for i in (1,2,3)]

    def _get_expected_joints_names(self) -> list[str]:
        """ which will be called in the (inherented) method,
        `_validate_consistency_with_src_URDF`
        """
        return [
            *self._gen_joint_names_ordered_1_2_3("lhs"),
            *self._gen_joint_names_ordered_1_2_3("rhs"),
        ]


    def _enforce_naming_convention(self):
        print("renaming the links according to the naming convention...")
        joint_ptr_list_dict = dict()
        for xhs in ("lhs", "rhs"):
            joint_ptr_list_dict[xhs] = grab_expected_joints_handle(
                self.urdf_root,
                self._gen_joint_names_ordered_1_2_3(xhs)
            )

        desired_body_link_name = "body"
        name_body = joint_ptr_list_dict["lhs"][0].find("parent").attrib["link"]
        if name_body != desired_body_link_name:
            print(f"   renaming the top-level non-dummy link [{name_body}]  as [{desired_body_link_name}]")
            # TODO checking whether it's really non-dummy
            rename_link(self.urdf_root, link_name_old=name_body, link_name_new=desired_body_link_name)

        for xhs in ("lhs", "rhs"):
            xhs_joint_ptr_list = joint_ptr_list_dict[xhs]
            # JIC watch out for the index offset

            # xhs_j1's child = xhs_inner_beam, 
            # xhs_j2's child = xhs_outer_beam.
            # xhs_j3's child = xhs_claw
            for (new_name, helper_joint_ptr) in zip(("inner_beam","outer_beam","claw"), xhs_joint_ptr_list):
                old_name = helper_joint_ptr.find("child").get("link")
                print(f"   renaming link [{old_name}]  as [{xhs}_{new_name}]")
                rename_link(self.urdf_root, old_name, xhs+"_"+new_name) # no namespace prefix yet

        # Wowww wow, joint_state_publisher has some strange behavior. 
        # So let's get around it by setting the mimic joints as "continuous" instead!
        # This is okay because their joint angles are implied by the kinematic loop and the angle of
        # the active joints, which shall already have the joint (position) limits.
        mimic_joint_handle_list = grab_expected_joints_handle(
            self.urdf_root, 
            self.mimic_joint_name_list
        )
        for mimic_joint_ptr in mimic_joint_handle_list:
            if mimic_joint_ptr.get("type") == "revolute":
                mimic_joint_ptr.attrib["type"] = "continuous"
            # TODO implement this in urdf_kit (I guess many projects also need this)

    def _inject_ns_aware_elems(self):
        print("injecting namespace-aware elements...")
        print("   working on the <mimic> elements")
        mimic_joint_handle_list = grab_expected_joints_handle(
            self.urdf_root, 
            self.mimic_joint_name_list
        )

        # mimimic flag, note that i can be out-of-order but that's fine!
        for i, joint_elem in enumerate(mimic_joint_handle_list):
            mimic_elem = ET.SubElement(
                joint_elem,
                "mimic", 
                joint = r"${ns}/"+self.mimic_spec_list[i].src,
                multiplier = f"{self.mimic_spec_list[i].multiplier:.0f}",
                offset = "0"
            )

        print("   working on the <transmission> elements")
        for actuator_joint_data in self.actuation_spec._active_joint_list:
            joint_name = r'${ns}/'+actuator_joint_data.joint
            joint_ctr_mode = actuator_joint_data.mode
            transmission_elem = make_simple_transmission_elem(joint_name, joint_ctr_mode)
            transmission_elem
            self.urdf_root.append(transmission_elem)
    
    def freeze_joints(self):
        if not self.has_frozen_output:
            print("Ignoring this call!")
            return
        else:
            freeze_angle_global = self.xacro_output_list[1].angle
        # current limitation
        if len(self.actuation_spec._active_joint_list) > 1:
            raise NotImplementedError("Currently we assume the gripper has only 1 DoF")
        
        # this works for any number of transmission elements
        remove_subelement_by_tag(self.urdf_root, "transmission")

        # the active joint
        if len(self.actuation_spec._active_joint_list) == 1:
            joint_name = self.actuation_spec._active_joint_list[0].joint
            joint_name = r"${ns}/"+joint_name
            print("  freezing joint [",joint_name,"] with angle [", freeze_angle_global, "rad ]")
            joint_elem = grab_expected_joints_handle(self.urdf_root, [joint_name])[0]
            fix_revolute_joint(joint_elem, freeze_angle_global)
        
        # passive joints
        joint_name_list = [r"${ns}/"+joint_spec.to for joint_spec in self.actuation_spec._passive_joint_list]
        joint_elem_list = grab_expected_joints_handle(self.urdf_root, joint_name_list)
        for i, joint_elem in enumerate(joint_elem_list):
            joint_spec = self.actuation_spec._passive_joint_list[i]
            angle = freeze_angle_global * joint_spec.multiplier
            print("  freezing joint [",joint_name_list[i],"] with angle [", angle, "rad ]")
            fix_revolute_joint(joint_elem, angle)
        
        print("now merging fixed joints upstream")
        robot = kinematic_tree(self.urdf_root)
        robot.merge_fixed_joints() # no whitelist
        # print(robot.links.keys())
        robot.print_graph(link_sorting='depth_first')
        
        
        

    def _make_targets(self):
        print(f"outputting the xacro target...")
        self.xacro_output_list[0].make_target(xml_root = self.urdf_root)

        if self.has_frozen_output:
            print(f"working on the frozen URDF target...")
            self.freeze_joints()
            self.xacro_output_list[1].make_target(xml_root = self.urdf_root)
