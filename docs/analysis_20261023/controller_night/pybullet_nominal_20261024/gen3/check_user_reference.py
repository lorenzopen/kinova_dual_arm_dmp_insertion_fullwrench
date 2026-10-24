import ast,json,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent
def methods(path):
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CartesianImpedanceController')
    result={}
    for n in cls.body:
        if not isinstance(n,ast.FunctionDef):continue
        if isinstance(n.body[0],ast.Expr) and isinstance(n.body[0].value,ast.Constant) and isinstance(n.body[0].value.value,str):n.body=n.body[1:]
        result[n.name]=ast.dump(n,include_attributes=False)
    return result
original=methods(HERE/'lab_source/kinova_controller_original.py')
reference=methods(HERE/'lab_source/user_supplied_methods.py')
checks={name:original[name]==value for name,value in reference.items()}
assert all(checks.values()),checks
result=dict(user_reference='Lorenzo supplied verbatim pure controller source and node contract in chat',
    executable_method_AST_equal=checks,
    original_source_sha256=hashlib.sha256((HERE/'lab_source/kinova_controller_original.py').read_bytes()).hexdigest(),
    boundary='Confirms supplied compute_torque and estimate_wrench executable bodies; not deployed binary/full-file hash verification',
    explicit_contract=dict(torque_sensor='minus previous applied motor torque',publish_hz=250,
        friction_scale=0.,home_seconds_launch=8.,cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3))
(HERE/'user_reference_check.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
