"""Exercise the actual parameter-selection method without ROS or hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS


def method():
    path = Path(__file__).parents[2] / 'src/patrol_manager/patrol_manager/door_patrol_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'configure_goal_tolerance')
    class Parameter:
        def __init__(self, name, value):
            self.name, self.value = name, value
        def to_parameter_msg(self):
            return self
    scope = {'Parameter': Parameter, 'SetParametersAtomically': NS(Request=NS)}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), scope)
    return scope[fn.name]


def test_blue_double_red_original_and_no_goal_dispatch():
    requests = []
    pending = object()
    def call(request):
        requests.append({p.name: p.value for p in request.parameters})
        return pending
    params = dict(nav2_xy_tolerance=.05, nav2_yaw_tolerance=.087, blue_tolerance_multiplier=2.)
    manager = NS(mission=NS(points=[NS(type='blue'), NS(type='red')]), index=0,
                 get_parameter=lambda key:NS(value=params[key]),
                 tolerance_client=NS(call_async=call))
    configure = method()
    configure(manager)
    assert manager.state == 'CONFIGURING_GOAL' and manager.tolerance_future is pending
    assert requests[-1] == {'general_goal_checker.xy_goal_tolerance': .1,
                            'general_goal_checker.yaw_goal_tolerance': .174}
    manager.index = 1
    configure(manager)
    assert requests[-1] == {'general_goal_checker.xy_goal_tolerance': .05,
                            'general_goal_checker.yaw_goal_tolerance': .087}
