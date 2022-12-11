from urdf_kit.automation import export_target_spec
from pathlib import Path
this_dir = Path(__file__).resolve().parent
import sys
gripper_module_dir = (this_dir/".."/"utility-general").resolve()
sys.path.append(str(gripper_module_dir))
from gripper_urdf import frozen_gripper_urdf_output
from gripper_urdf import gripper_xacro_export
from gripper_model import gripper_actuation_concept

import numpy as np

target = export_target_spec(
    target_full_path=this_dir/".."/"gripper_description"/"urdf"/"gripper.reusable.xacro",
    reusable_macro_name="mk_parallel_gripper"
)

target_frozen = frozen_gripper_urdf_output(
    target_full_path=this_dir/".."/"gripper_description"/"urdf"/"gripper_0dof.reusable.xacro", 
    reusable_macro_name="mk_parallel_gripper_with_0dof",
    freeze_angle = np.deg2rad(-45)
)

import yaml
with open(this_dir/"config_gripper_actuation.yaml","r") as f:
    data = yaml.safe_load(f)
gripper_actuation_spec = gripper_actuation_concept(data)

worker = gripper_xacro_export(
    urdf_xacro_output_spec  = target, 
    OnshapeToRobotProjectDir = this_dir,
    actuation_spec=gripper_actuation_spec,
    extra_urdf_output_frozen_angle=target_frozen
)

worker.postprocessing_chain()
