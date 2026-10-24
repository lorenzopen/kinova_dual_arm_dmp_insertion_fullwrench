"""Preserve and extract hardware-independent classes verbatim, never import ROS/Kortex."""
import ast,json,hashlib
from pathlib import Path
from datetime import datetime,timezone
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[4]
SOURCE=Path('C:/Users/Lorenzo/Downloads/kinova_controller_pkgs-20261012T072226Z-1-001/kinova_controller_pkgs/kinova_control/scripts/kinova_controller.py')
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
    dest=HERE/'lab_source';dest.mkdir(exist_ok=True)
    raw=SOURCE.read_bytes();text=raw.decode('utf-8-sig');tree=ast.parse(text)
    snapshot=dest/'kinova_controller_original.py'
    if snapshot.exists():assert snapshot.read_bytes()==raw,'Do not overwrite a different frozen controller'
    else:snapshot.write_bytes(raw)
    names={'ControlMode','angle_diff','URDFStribeckFriction','OnlineKalmanState','JointSpaceController','CartesianImpedanceController'}
    segments=[(n,ast.get_source_segment(text,n)) for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in names]
    assert {n.name for n,s in segments}==names
    generated='# Verbatim source segments from the frozen laboratory controller.\nimport math\nfrom enum import Enum, auto\nimport numpy as np\nimport pinocchio as pin\n\n'+'\n\n'.join(s for n,s in segments)+'\n'
    (dest/'controller_math.py').write_text(generated,encoding='utf-8')
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='RealRobotImpedance')
    defaults={}
    for call in ast.walk(node):
        if isinstance(call,ast.Call) and isinstance(call.func,ast.Attribute) and call.func.attr=='declare_parameter' and len(call.args)>1:
            try:defaults[ast.literal_eval(call.args[0])]=ast.literal_eval(call.args[1])
            except (ValueError,TypeError):pass
    record=dict(created_utc=datetime.now(timezone.utc).isoformat(),source=str(SOURCE),source_sha256=sha(raw),
        source_lines=len(text.splitlines()),matches_user_summary='2092 lines; Cartesian class462, compute_torque563, defaults and main loop agree',
        deployed_NVIDIA_hash_verified=False,extraction='AST selects pure classes/functions, source text preserved verbatim; ROS/Kortex node is not imported',
        segments=[dict(name=n.name,start_line=n.lineno,end_line=n.end_lineno,sha256=sha(s.encode())) for n,s in segments],
        generated_sha256=sha(generated.encode()),literal_node_defaults=defaults,
        friction_note='Local robot.launch.py enables friction but does not override source friction_scale default0. Missing deployed gen3_bare.urdf is immaterial only at scale0; do not invent nonzero compensation.')
    (dest/'provenance.json').write_text(json.dumps(record,indent=2)+'\n')
    (dest/'robot_launch_original.py').write_bytes((ROOT/'launch/robot.launch.py').read_bytes())
    print(json.dumps(record,indent=2))
if __name__=='__main__':main()
