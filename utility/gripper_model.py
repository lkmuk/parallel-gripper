import re
import xml.etree.ElementTree as ET
from collections import namedtuple, OrderedDict

from urdf_kit.edit_joints import grab_expected_joints_handle
from urdf_kit.misc import format_then_write, print_urdf
from urdf_kit.edit_links import rename_link, purge_nonprimitive_collision_geom

ctr_spec_entry = namedtuple("ctr_spec_entry", ["joint","mode"])
passive_entry = namedtuple("passive_entry", ["src","to","multiplier", "gearing"])
valid_control_mode = "pos", # TODO extension! with vel, eff

def assert_naming_convention(str_under_test):
    assert isinstance(str_under_test, str), "You gave ["+str(str_under_test)+"]."
    assert re.match(r"[lr]hs_j[1-4]",str_under_test), "please adhere to the naming convention! You gave ["+str_under_test+ "]. Please check the doc!"

# ??? what about ?hs_j4 ??? (not found in Onshape-to-Robot export!)
# need to add that first? 

# zero-position!

# assumption! j4 never active!!!!


# no checks on lhs???
class gripper_actuation_concept:
    def __init__(self, yaml_dom: dict):
        
        self._active_joint_list = []
        self._active_joint_name_set = set() # caching
        if "control" not in yaml_dom.keys():
            raise ValueError("Seems you forgot specifying the actuators...")
        else:
            for control in yaml_dom["control"]:
                assert_naming_convention(control["joint"])
                assert control["joint"] not in self._active_joint_name_set, "Duplicated entry for joint ["+str(control["joint"])+"! Each joint can only have one control mode!"
                assert control["mode"] in valid_control_mode, "recognized mode(s) are : "+str(valid_control_mode)+", you gave"+control["mode"]
                self._active_joint_name_set.add(control["joint"])
                self._active_joint_list.append(ctr_spec_entry(**control))
        

        self._passive_joint_list = []
        self._passive_joint_name_set = set() # caching
        if "mimic" not in yaml_dom.keys():
            raise ValueError("Didn't find any passive joints. This is really weird...")
        else:
            for entry in yaml_dom["mimic"]:
                assert_naming_convention(entry["src"])
                assert_naming_convention(entry["to"])
                assert isinstance(entry["gearing"], bool)
                assert entry["multiplier"] in (+1, -1), "This script is intended only for parallel-link 4-bar!"
                assert entry["to"] not in self._active_joint_name_set, "joint ["+entry["src"]+"] is actively controlled!"
                assert entry["to"] not in self._passive_joint_name_set, "Duplicate mimic entry for joint ["+entry["to"]+"]!"
                assert entry["src"] in self._active_joint_name_set, "joint ["+entry["to"]+"] is NOT actively controlled!" # really necessary?
                # finally!
                self._passive_joint_list.append(passive_entry(**entry))
                self._passive_joint_name_set.add(entry["to"])
  
        assert len(self._active_joint_name_set) + len(self._passive_joint_list) == (2*3), "Invalid config! Check the naming convention also J4 shall not be there!"
    
    def print(self):
        print("-"*30+"\nactive joints:")
        for entry in self._active_joint_list:
            print(entry)
        print("-"*30+"\npassive joints:")
        for entry in self._passive_joint_list:
            print(entry)
    
class gripper_urdf_from_Onshape:
    def __init__(self, urdf_root: ET.ElementTree, actuation_spec : gripper_actuation_concept):
        """
        URDF export: 
            mimic tags (not really to do with j4 *2 !!!) + transmission tags (for all active joints)
        SDF "export: 
            the URDF export (the mimic tags will be ignored anyways) + .gazebo (gearing + geometric constraint, i.e. xhs_j4)
        """
        self.ran_once = False

        # validate it first! 
        # I will just scan for ALL of the expected **joint** names. then also rename the corresponding links.
        # remarks: extra links and joints are fine

        # # overcomplicated ...
        # wishlist_lhs = [f"lhs_j{i}" for i in range(1, 4+1)]
        # wishlist_rhs = [f"rhs_j{i}" for i in range(1, 4+1)]
        # self.joint_handles_lhs = grab_expected_joints_handle(urdf_root, wishlist_lhs)
        # self.joint_handles_rhs = grab_expected_joints_handle(urdf_root, wishlist_rhs)

        self.actuation_spec = actuation_spec        
        self.mimic_spec_list = actuation_spec._passive_joint_list
        self.mimic_joint_name_list = [spec.to for spec in self.mimic_spec_list]

        self.robot = urdf_root
        self.mimic_joint_handle_list = grab_expected_joints_handle(self.robot, self.mimic_joint_name_list)


        # finally
        self._augment()

        print_urdf(self.robot)

        # the gazebo stuff --- gearing and the extra link will be provided as supplementary files (based on the configuration?????) instead...
        # TODO inject xacro include into the urdf !??



    def _augment(self):
        assert not self.ran_once, "Running more than once will need to duplication!"
        #
        # transmission flag
        # mimimic flag
        for i, joint_elem in enumerate(self.mimic_joint_handle_list):
            mimic_elem = ET.SubElement(
                joint_elem,
                "mimic", 
                joint = self.mimic_spec_list[i].src,
                multiplier = f"{self.mimic_spec_list[i].multiplier:.0f}",
                offset = "0"
            ) 

            # Wowww wow, joint_state_publisher has some strange behavior. 
            # So let's get around it by setting the mimic joints as "continuous" instead!
            # This is okay because their joint angles are implied by the kinematic loop and the angle of
            # the active joints, which shall already have the joint (position) limits.
            if joint_elem.get("type") == "revolute":
                joint_elem.attrib["type"] = "continuous"

        self.ran_once = True
        
        

    def write(self, outfile_path):
        format_then_write(self.robot, outfile_path)
        
    def purge_collision_mesh(self, whitelist: list[str] = []):
    	purge_nonprimitive_collision_geom(self.robot, whitelist)
    		


if __name__ == "__main__":

    from sys import argv
    from pathlib  import Path
    this_dir = Path(__file__).resolve().parent

    import yaml
    with open(this_dir/"config_gripper_actuation.yaml","r") as f:
        data = yaml.safe_load(f)
    my_spec = gripper_actuation_concept(data)
    my_spec.print()

    import xml.etree.ElementTree as ET
    fpath_urdf_original = this_dir/"robot.urdf"
    fpath_urdf_new = this_dir/"robot_augmented.urdf"
    

    editor = gripper_urdf_from_Onshape(
        ET.parse(fpath_urdf_original).getroot(), 
        my_spec
    )
    if len(argv) > 0:
    	editor.purge_collision_mesh(whitelist=argv[1:])
    editor.write(fpath_urdf_new)

