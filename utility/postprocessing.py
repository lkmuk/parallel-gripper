from urdf_kit.automation import export_target_spec
from pathlib import Path
this_dir = Path(__file__).resolve().parent
import sys
gripper_module_dir = (this_dir/".."/"utility-general").resolve()
sys.path.append(str(gripper_module_dir))
from gripper_urdf import gripper_xacro_export
from gripper_model import gripper_actuation_concept


target = export_target_spec(
    target_full_path=this_dir/".."/"gripper_description"/"urdf"/"gripper.reusable.xacro",
    reusable_macro_name="mk_parallel_gripper"
)

import yaml
with open(this_dir/"config_gripper_actuation.yaml","r") as f:
    data = yaml.safe_load(f)
gripper_actuation_spec = gripper_actuation_concept(data)

worker = gripper_xacro_export(
    urdf_xacro_output_spec  = target, 
    OnshapeToRobotProjectDir = this_dir,
    actuation_spec=gripper_actuation_spec
)

worker.postprocessing_chain()
