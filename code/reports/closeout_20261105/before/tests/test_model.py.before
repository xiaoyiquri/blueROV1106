import numpy as np
import pytest

from bluerov2 import parameters, heavy_reference_parameters, bluerov2_dynamics, rk4_step
from bluerov2.lbts import default_lbts_options, make_rk4_linearizer, evaluate_rk4_linearization
from bluerov2.kprm import Edge, Roadmap, load_trajectory_graph, save_trajectory_graph


def test_standard_model_dimensions_and_rank():
    p=parameters();m=p.model
    assert m.num_thrusters==6 and m.allocation.shape==(6,6)
    assert np.linalg.matrix_rank(m.allocation)==5
    assert np.allclose(m.allocation[:,0],np.r_[m.thruster_direction[:,0],np.cross(m.thruster_position[:,0],m.thruster_direction[:,0])])
    assert bluerov2_dynamics(np.zeros(12),np.zeros(6),m).shape==(12,)


def test_linearizer_shapes_and_rk4():
    p=parameters();o=default_lbts_options(p.model);lin=make_rk4_linearizer(p.model,o.linearization)
    x=np.zeros(12);u=np.zeros(6);xn,A,B=evaluate_rk4_linearization(lin,x,u,.01)
    assert xn.shape==(12,1) or xn.shape==(12,)
    assert A.shape==(12,12) and B.shape==(12,6)
    assert rk4_step(x,u,.01,p.model).shape==(12,)
    fd=make_rk4_linearizer(p.model,type("S",(),{"prefer_casadi":False,"allow_finite_difference":True,"finite_difference_step":1e-6})())
    _,Af,Bf=evaluate_rk4_linearization(fd,x,u,.01)
    assert np.allclose(A,Af,rtol=2e-4,atol=2e-6) and np.allclose(B,Bf,rtol=2e-4,atol=2e-6)


def test_heavy_reference_is_not_executable(tmp_path):
    heavy=heavy_reference_parameters()
    assert heavy.model.num_thrusters==8
    with pytest.raises(ValueError):
        heavy.model.validate()
    x=np.linspace(-.1,.1,12);f=np.linspace(-2,2,8)
    assert bluerov2_dynamics(x,f,heavy.model).shape==(12,)


def test_old_mat_file_rejected(tmp_path):
    with pytest.raises(ValueError,match="MAT"):
        load_trajectory_graph(tmp_path/"legacy.mat")
