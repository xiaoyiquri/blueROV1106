"""BlueROV2 model and planning parameters.

The module is deliberately data oriented.  Every public numerical interface uses
column vectors represented by one dimensional NumPy arrays and trajectory arrays
are ``(state, sample)`` / ``(control, interval)``.  The default model is the
factory six-thruster BlueROV2; ``heavy_reference_parameters`` is retained solely
for MATLAB migration tests.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any
import copy
import math
import numpy as np


def _skew(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float).reshape(3)
    return np.array([[0., -a[2], a[1]], [a[2], 0., -a[0]],
                     [-a[1], a[0], 0.]])


def _mass_matrix(m: float, inertia_cg: np.ndarray, rg: np.ndarray,
                 added_mass: np.ndarray) -> np.ndarray:
    S = _skew(rg)
    io = inertia_cg - m * S @ S
    mrb = np.block([[m * np.eye(3), -m * S], [m * S, io]])
    return mrb + np.diag(added_mass)


@dataclass
class ModelParameters:
    # Rigid body and low speed hydrodynamic values copied from the MATLAB source.
    m: float = 11.5
    Ig: np.ndarray = field(default_factory=lambda: np.diag([.16, .16, .16]))
    W: float = 112.8
    B: float = 114.8
    rg: np.ndarray = field(default_factory=lambda: np.array([0., 0., .020]))
    rb: np.ndarray = field(default_factory=lambda: np.zeros(3))
    added_mass: np.ndarray = field(default_factory=lambda: np.array([5.50, 12.70, 14.60, .12, .12, .12]))
    linear_damping: np.ndarray = field(default_factory=lambda: np.array([4.03, 6.22, 5.18, .07, .07, .07]))
    quadratic_damping: np.ndarray = field(default_factory=lambda: np.array([18.18, 21.66, 36.99, 1.55, 1.55, 1.55]))
    smooth_abs_eps: float = 1.e-4
    thruster_position: np.ndarray | None = None
    thruster_direction: np.ndarray | None = None
    thrust_min: np.ndarray | None = None
    thrust_max: np.ndarray | None = None
    name: str = "BlueROV2 standard six-thruster"
    geometry_status: str = (
        "unverified motor-to-position mapping: inherited horizontal y signs "
        "conflict with FRD port/starboard labels; horizontal arm dimensions "
        "follow the source Heavy model and vertical arm dimensions are estimates; "
        "motor numbering, geometry and force signs require vehicle verification"
    )
    source_urls: tuple[str, ...] = (
        "https://bluerobotics.com/learn/bluerov2-software-setup/",
        "https://bluerobotics.com/learn/bluerov2-assembly/",
        "https://ardupilot.ardupilot.org/sub/docs/sub-frames.html",
    )
    mass_matrix: np.ndarray = field(init=False)
    mass_matrix_inverse: np.ndarray = field(init=False)
    allocation: np.ndarray = field(init=False)
    allocation_pseudoinverse: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.Ig = np.asarray(self.Ig, float).reshape(3, 3)
        self.rg = np.asarray(self.rg, float).reshape(3)
        self.rb = np.asarray(self.rb, float).reshape(3)
        self.added_mass = np.asarray(self.added_mass, float).reshape(6)
        self.linear_damping = np.asarray(self.linear_damping, float).reshape(6)
        self.quadratic_damping = np.asarray(self.quadratic_damping, float).reshape(6)
        if self.thruster_position is None or self.thruster_direction is None:
            raise ValueError("thruster_position and thruster_direction are required")
        self.thruster_position = np.asarray(self.thruster_position, float)
        self.thruster_direction = np.asarray(self.thruster_direction, float)
        if self.thruster_position.shape != (3, self.num_thrusters):
            raise ValueError(f"thruster_position must have shape (3,{self.num_thrusters})")
        if self.thruster_direction.shape != (3, self.num_thrusters):
            raise ValueError(f"thruster_direction must have shape (3,{self.num_thrusters})")
        norms = np.linalg.norm(self.thruster_direction, axis=0)
        if np.any(norms == 0) or np.any(~np.isclose(norms, 1.0, atol=1e-10)):
            raise ValueError("thruster directions must be nonzero unit vectors")
        if self.thrust_min is None:
            self.thrust_min = -34.0 * np.ones(self.num_thrusters)
        if self.thrust_max is None:
            self.thrust_max = 34.0 * np.ones(self.num_thrusters)
        self.thrust_min = np.asarray(self.thrust_min, float).reshape(self.num_thrusters)
        self.thrust_max = np.asarray(self.thrust_max, float).reshape(self.num_thrusters)
        if np.any(self.thrust_min > self.thrust_max):
            raise ValueError("thrust_min must not exceed thrust_max")
        self.allocation = np.empty((6, self.num_thrusters), float)
        for i in range(self.num_thrusters):
            d = self.thruster_direction[:, i]
            r = self.thruster_position[:, i]
            self.allocation[:, i] = np.r_[d, np.cross(r, d)]
        self.mass_matrix = _mass_matrix(self.m, self.Ig, self.rg, self.added_mass)
        self.mass_matrix_inverse = np.linalg.solve(self.mass_matrix, np.eye(6))
        self.allocation_pseudoinverse = np.linalg.pinv(self.allocation)

    @property
    def num_thrusters(self) -> int:
        if self.thruster_position is None:
            return 0
        return int(np.asarray(self.thruster_position).shape[1])

    def validate(self) -> None:
        if self.num_thrusters != 6:
            raise ValueError(f"default execution requires six thrusters, got {self.num_thrusters}")
        if self.allocation.shape != (6, 6):
            raise ValueError("allocation must be (6,6) for the standard vehicle")

    def copy(self) -> "ModelParameters":
        return copy.deepcopy(self)


@dataclass
class OCPParameters:
    N: int = 20
    Tmin: float = .40
    Tmax: float = 8.0
    v_nominal: float = .45
    w_time: float = 1.0
    R: np.ndarray = field(default_factory=lambda: 2.e-4 * np.eye(6))
    w_delta_u: float = 2.e-5
    max_linear_velocity: np.ndarray = field(default_factory=lambda: np.array([1., 1., .7]))
    max_angular_velocity: np.ndarray = field(default_factory=lambda: np.deg2rad([50., 50., 70.]))
    max_pitch: float = math.radians(75.)
    ipopt_max_iter: int = 600
    acceptable_tol: float = 1.e-5
    verbose: bool = False

    def __post_init__(self):
        self.R = np.asarray(self.R, float)
        self.max_linear_velocity = np.asarray(self.max_linear_velocity, float).reshape(3)
        self.max_angular_velocity = np.asarray(self.max_angular_velocity, float).reshape(3)


@dataclass
class PRMParameters:
    rng_seed: int = 0
    num_positions: int = 80
    min_position_separation: float = 1.4
    max_sample_attempts: int = 30000
    sample_speed: float = .25
    sample_angular_rate: float = math.radians(8.)
    roll_range: tuple[float, float] = (math.radians(-10), math.radians(10))
    pitch_range: tuple[float, float] = (math.radians(-15), math.radians(15))
    connection_radius: float = 1.7
    min_edge_position_distance: float = .01
    source_direction_cosine: float = math.cos(math.radians(10))
    target_direction_cosine: float = -.99
    direction_speed_tolerance: float = 1.e-8
    save_file: str = "trajectory_graph.pkl"


@dataclass
class QueryParameters:
    connection_radius: float = 2.5
    k_neighbors: int = 14
    try_direct_connection: bool = True
    verbose: bool = True


@dataclass
class World:
    position_bounds: np.ndarray = field(default_factory=lambda: np.array([[-4., 4.], [-2.5, 2.5], [.5, 4.]]))
    robot_radius: float = .22
    collision_step: float = .08
    obstacles: list[dict[str, Any]] = field(default_factory=lambda: [
        {"type": "sphere", "center": np.array([-1., 0., 2.5]), "radius": .4},
        {"type": "box", "center": np.array([1., -1., 2.3]), "size": np.array([1., .6, .5])},
    ])

    def __post_init__(self):
        self.position_bounds = np.asarray(self.position_bounds, float).reshape(3, 2)
        normal = []
        for o in self.obstacles:
            q = dict(o)
            q["type"] = str(q["type"]).lower()
            q["center"] = np.asarray(q["center"], float).reshape(3)
            if q["type"] == "sphere": q["radius"] = float(q["radius"])
            elif q["type"] == "box": q["size"] = np.asarray(q["size"], float).reshape(3)
            else: raise ValueError(f"unknown obstacle type {q['type']}")
            normal.append(q)
        self.obstacles = normal


@dataclass
class Parameters:
    model: ModelParameters
    ocp: OCPParameters = field(default_factory=OCPParameters)
    prm: PRMParameters = field(default_factory=PRMParameters)
    world: World = field(default_factory=World)
    query: QueryParameters = field(default_factory=QueryParameters)
    metric_weights: np.ndarray = field(default_factory=lambda: np.array([1,1,1,.30,.30,.30,.40,.40,.40,.15,.15,.15], float))

    def __post_init__(self):
        self.metric_weights = np.asarray(self.metric_weights, float).reshape(12)
        if self.ocp.R.shape != (self.model.num_thrusters, self.model.num_thrusters):
            raise ValueError("OCP R must have one row/column per thruster")

    def copy(self) -> "Parameters":
        return copy.deepcopy(self)


def _standard_geometry() -> tuple[np.ndarray, np.ndarray]:
    """Factory order [1,2,3,4,5,6], with NED force directions.

    Motor numbers and the 45-degree horizontal orientation are from the official
    Blue Robotics diagrams.  The public drawings do not specify a body-reference
    coordinate table; the horizontal arm values mirror the source project's
    measured/assumed values and the two vertical y locations are the corresponding
    standard-frame estimate.  They are explicit parameters so a measured vehicle
    can replace them without touching dynamics.
    """
    c = math.sqrt(.5)
    # 1 front-starboard, 2 front-port, 3 rear-starboard, 4 rear-port;
    # 5 port and 6 starboard vertical.  Positive f is force on the vehicle.
    # WARNING: inherited horizontal y signs below conflict with these intended
    # FRD labels. Keep this estimated geometry explicit until motor mapping is
    # verified on the vehicle; it is not an actuator-channel calibration.
    r = np.array([
        [.156, .156, -.156, -.156, 0., 0.],
        [-.111, .111, -.111, .111, -.218, .218],
        [.085, .085, .085, .085, 0., 0.],
    ])
    # The standard frame's front pair and rear pair are mounted with opposite
    # axial orientations.  These signs agree with the official Vectored mixer
    # orientation (the mixer coefficients themselves are not used as force
    # magnitudes).  Vertical positive force is +body-z (down in NED).
    d = np.array([
        [-c, -c, c, c, 0., 0.],
        [ c, -c, c, -c, 0., 0.],
        [0., 0., 0., 0., 1., 1.],
    ])
    return r, d


def parameters() -> Parameters:
    r, d = _standard_geometry()
    p = Parameters(ModelParameters(thruster_position=r, thruster_direction=d))
    p.model.validate()
    return p


def heavy_reference_parameters() -> Parameters:
    """The original eight-input Heavy geometry for migration-only comparisons."""
    c = math.sqrt(.5)
    r = np.array([[.156,.156,-.156,-.156,.120,.120,-.120,-.120],
                  [-.111,.111,-.111,.111,-.218,.218,.218,-.218],
                  [.085,.085,.085,.085,0.,0.,0.,0.]])
    d = np.array([[c,c,c,c,0,0,0,0],[-c,c,c,-c,0,0,0,0],[0,0,0,0,1,1,1,1]])
    ocp = OCPParameters(R=2.e-4*np.eye(8))
    prm = PRMParameters()
    return Parameters(ModelParameters(thruster_position=r, thruster_direction=d,
                                      thrust_min=-34*np.ones(8), thrust_max=34*np.ones(8),
                                      name="BlueROV2 Heavy eight-thruster reference"), ocp=ocp, prm=prm)


def wrap_to_pi(a: np.ndarray | float) -> np.ndarray | float:
    return np.arctan2(np.sin(a), np.cos(a))


def as_namespace(p: Parameters) -> SimpleNamespace:
    """Compatibility helper for callers that used MATLAB-like attribute access."""
    return SimpleNamespace(model=p.model, ocp=p.ocp, prm=p.prm, world=p.world,
                           query=p.query, metric=SimpleNamespace(weights=p.metric_weights))


def model_metadata(model: ModelParameters) -> dict[str, Any]:
    return {
        "model_version": "python-six-thruster-v1",
        "model_name": model.name,
        "num_thrusters": model.num_thrusters,
        "thruster_order": [1,2,3,4,5,6] if model.num_thrusters == 6 else list(range(1, model.num_thrusters+1)),
        "state_order": ["N","E","D","roll","pitch","yaw","u","v","w","p","q","r"],
        "coordinate_system": "NED; body x forward, y starboard, z down",
        "units": {"position": "m", "attitude": "rad", "linear_velocity": "m/s", "angular_velocity": "rad/s", "thrust": "N"},
        "geometry_status": model.geometry_status,
        "allocation_rank": int(np.linalg.matrix_rank(model.allocation)),
        "allocation_singular_values": np.linalg.svd(model.allocation, compute_uv=False).tolist(),
    }


def allocation_report(model: ModelParameters) -> dict[str, Any]:
    """Return rank, singular values and the wrench-space left nullspace."""
    U, singular, _ = np.linalg.svd(model.allocation, full_matrices=True)
    rank = int(np.linalg.matrix_rank(model.allocation))
    null = U[:, rank:]
    return {"rank": rank, "singular_values": singular, "wrench_left_nullspace": null,
            "uncontrollable_wrench_directions": null.T}
