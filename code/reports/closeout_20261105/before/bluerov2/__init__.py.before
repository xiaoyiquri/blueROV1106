"""BlueROV2 public API; planning modules load only when requested."""
from importlib import import_module
import sys
from types import ModuleType

_EXPORTS = {'parameters': ('model', 'parameters'), 'heavy_reference_parameters': ('model', 'heavy_reference_parameters'), 'Parameters': ('model', 'Parameters'), 'ModelParameters': ('model', 'ModelParameters'), 'model_metadata': ('model', 'model_metadata'), 'allocation_report': ('model', 'allocation_report'), 'bluerov2_dynamics': ('dynamics', 'bluerov2_dynamics'), 'rk4_step': ('dynamics', 'rk4_step'), 'Edge': ('kprm', 'Edge'), 'Roadmap': ('kprm', 'Roadmap'), 'mean_control': ('kprm', 'mean_control'), 'build_kinodynamic_prm': ('kprm', 'build_kinodynamic_prm'), 'attach_query_endpoints': ('kprm', 'attach_query_endpoints'), 'query_kinodynamic_prm': ('kprm', 'query_kinodynamic_prm'), 'weighted_state_distance': ('kprm', 'weighted_state_distance'), 'save_trajectory_graph': ('kprm', 'save_trajectory_graph'), 'load_trajectory_graph': ('kprm', 'load_trajectory_graph'), 'sample_blue_rov2_states': ('kprm', 'sample_blue_rov2_states'), 'collision_filtered_adjacency': ('kprm', 'collision_filtered_adjacency'), 'dijkstra_directed': ('kprm', 'dijkstra_directed'), 'get_kinodynamic_edge': ('kprm', 'get_kinodynamic_edge'), 'default_lbts_options': ('lbts', 'default_lbts_options'), 'lbts': ('lbts', 'lbts'), 'propagate_belief': ('lbts', 'propagate_belief'), 'monte_carlo_path': ('lbts', 'monte_carlo_path'), 'measurement_model_at_state': ('lbts', 'measurement_model_at_state'), 'chance_collision_probability': ('lbts', 'chance_collision_probability'), 'make_rk4_linearizer': ('lbts', 'make_rk4_linearizer'), 'evaluate_rk4_linearization': ('lbts', 'evaluate_rk4_linearization'), 'belief_dominated': ('lbts', 'belief_dominated'), 'execute_lbts_plan': ('execution', 'execute_lbts_plan'), 'save_result': ('io', 'save_result'), 'load_result': ('io', 'load_result'), 'inspect_legacy_mat_file': ('io', 'inspect_legacy_mat_file')}
__all__ = list(_EXPORTS)

def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(name)
    module, attribute = _EXPORTS[name]
    value = getattr(import_module('.'+module, __name__), attribute)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))


class _PublicAPI(ModuleType):
    def __getattribute__(self, name):
        value = super().__getattribute__(name)
        # Importing the same-named submodule must not replace the historical
        # package-level ``lbts`` function with a module object.
        if name == 'lbts' and isinstance(value, ModuleType):
            return value.lbts
        return value


sys.modules[__name__].__class__ = _PublicAPI
