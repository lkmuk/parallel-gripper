import re
from collections import namedtuple, OrderedDict


ctr_spec_entry = namedtuple("ctr_spec_entry", ["joint","mode"])
passive_entry = namedtuple("passive_entry", ["src","to","multiplier", "gearing"])

from urdf_kit.edit_transmission import SUPPORTED_JOINT_CMD_MODE
valid_control_mode = SUPPORTED_JOINT_CMD_MODE

def assert_naming_convention(str_under_test):
    assert isinstance(str_under_test, str), "You gave ["+str(str_under_test)+"]."
    assert re.match(r"[lr]hs_j[1-4]",str_under_test), "please adhere to the naming convention! You gave ["+str_under_test+ "]. Please check the doc!"

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
        print("   active joints:")
        for entry in self._active_joint_list:
            print("      ", entry)
        print("   passive joints:")
        for entry in self._passive_joint_list:
            print("      ", entry)
    

# an example
if __name__ == "__main__":

    from sys import argv
    from pathlib  import Path
    this_dir = Path(__file__).resolve().parent

    import yaml
    with open(this_dir/".."/"utility"/"config_gripper_actuation.yaml","r") as f:
        data = yaml.safe_load(f)
    my_spec = gripper_actuation_concept(data)
    my_spec.print()

