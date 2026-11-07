"""Continuous six-DOF BlueROV2 dynamics and RK4 integration."""
from __future__ import annotations
import numpy as np
from .model import ModelParameters

try:
    import casadi as ca
except Exception:  # pragma: no cover - optional at import time
    ca = None


def _symbolic(value) -> bool:
    return ca is not None and isinstance(value, (ca.SX, ca.MX, ca.DM))


def _skew_any(a):
    return ca.vertcat(ca.horzcat(0, -a[2], a[1]),
                      ca.horzcat(a[2], 0, -a[0]),
                      ca.horzcat(-a[1], a[0], 0)) if _symbolic(a) else np.array(
        [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]], dtype=float)


def rotation_zyx(phi, theta, psi):
    cp, sp, ct, st, cs, ss = (np.cos(phi), np.sin(phi), np.cos(theta),
                              np.sin(theta), np.cos(psi), np.sin(psi))
    if _symbolic(cp):
        return ca.vertcat(ca.horzcat(ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss),
                          ca.horzcat(ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs),
                          ca.horzcat(-st, sp*ct, cp*ct))
    return np.array([[ct*cs, sp*st*cs-cp*ss, cp*st*cs+sp*ss],
                     [ct*ss, sp*st*ss+cp*cs, cp*st*ss-sp*cs],
                     [-st, sp*ct, cp*ct]], dtype=float)


def euler_rate_map(phi, theta):
    cp, sp, ct, tt = np.cos(phi), np.sin(phi), np.cos(theta), np.tan(theta)
    if _symbolic(cp):
        return ca.vertcat(ca.horzcat(1, sp*tt, cp*tt),
                          ca.horzcat(0, cp, -sp),
                          ca.horzcat(0, sp/ct, cp/ct))
    return np.array([[1, sp*tt, cp*tt], [0, cp, -sp], [0, sp/ct, cp/ct]], float)


def coriolis_from_mass(M, nu):
    v, w = nu[:3], nu[3:6]
    p = M[:3, :3] @ v + M[:3, 3:6] @ w
    h = M[3:6, :3] @ v + M[3:6, 3:6] @ w
    if _symbolic(nu):
        z = ca.MX.zeros(3, 3) if isinstance(nu, ca.MX) else ca.SX.zeros(3, 3)
        return ca.vertcat(ca.horzcat(z, -_skew_any(p)),
                          ca.horzcat(-_skew_any(p), -_skew_any(h)))
    return np.block([[np.zeros((3,3)), -_skew_any(p)],
                     [-_skew_any(p), -_skew_any(h)]])


def restoring_forces(phi, theta, model: ModelParameters):
    cp, sp, ct, st = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta)
    W, B = model.W, model.B
    xg, yg, zg = model.rg
    xb, yb, zb = model.rb
    vals = [(W-B)*st,
            -(W-B)*ct*sp,
            -(W-B)*ct*cp,
            -(yg*W-yb*B)*ct*cp + (zg*W-zb*B)*ct*sp,
            (zg*W-zb*B)*st + (xg*W-xb*B)*ct*cp,
            -(xg*W-xb*B)*ct*sp - (yg*W-yb*B)*st]
    if _symbolic(phi):
        return ca.vertcat(*vals)
    return np.asarray(vals, float)


def bluerov2_dynamics(x, f, model: ModelParameters):
    """Return ``xdot`` for state ``(12,)`` and thrust ``(nu,)``.

    The same expression accepts NumPy arrays and CasADi SX/MX values.  A six
    thruster model is required by the default application; the eight-thruster
    reference is still accepted for migration tests.
    """
    sym = _symbolic(x) or _symbolic(f)
    if not sym:
        x = np.asarray(x, float).reshape(-1)
        f = np.asarray(f, float).reshape(-1)
        if x.size != 12 or f.size != model.num_thrusters:
            raise ValueError(f"expected x=(12,), f=({model.num_thrusters},)")
    eta, nu = x[:6], x[6:12]
    phi, theta, psi = eta[3], eta[4], eta[5]
    R = rotation_zyx(phi, theta, psi)
    T = euler_rate_map(phi, theta)
    if sym:
        z3 = ca.MX.zeros(3,3) if isinstance(x, ca.MX) else ca.SX.zeros(3,3)
        J = ca.vertcat(ca.horzcat(R, z3), ca.horzcat(z3, T))
        eta_dot = J @ nu
        M = ca.DM(model.mass_matrix)
        Minv = ca.DM(model.mass_matrix_inverse)
        C = coriolis_from_mass(M, nu)
        eps = model.smooth_abs_eps
        abs_nu = ca.sqrt(nu*nu + eps*eps)
        d = ca.DM(model.linear_damping) + ca.times(ca.DM(model.quadratic_damping), abs_nu)
        damping = ca.times(d, nu)
        restoring = restoring_forces(phi, theta, model)
        tau = ca.DM(model.allocation) @ f
        return ca.vertcat(eta_dot, Minv @ (tau - C @ nu - damping - restoring))
    eta_dot = np.block([[R, np.zeros((3,3))], [np.zeros((3,3)), T]]) @ nu
    C = coriolis_from_mass(model.mass_matrix, nu)
    abs_nu = np.sqrt(nu*nu + model.smooth_abs_eps**2)
    damping = (model.linear_damping + model.quadratic_damping*abs_nu)*nu
    tau = model.allocation @ f
    nu_dot = model.mass_matrix_inverse @ (tau - C @ nu - damping - restoring_forces(phi, theta, model))
    return np.r_[eta_dot, nu_dot]


def rk4_step(x: np.ndarray, f: np.ndarray, dt: float, model: ModelParameters) -> np.ndarray:
    x = np.asarray(x, float).reshape(12)
    f = np.asarray(f, float).reshape(model.num_thrusters)
    k1 = bluerov2_dynamics(x, f, model)
    k2 = bluerov2_dynamics(x + dt*k1/2, f, model)
    k3 = bluerov2_dynamics(x + dt*k2/2, f, model)
    k4 = bluerov2_dynamics(x + dt*k3, f, model)
    return x + dt*(k1 + 2*k2 + 2*k3 + k4)/6


def numerical_jacobian(fun, x, h=1e-6):
    x = np.asarray(x, float).reshape(-1)
    y = np.asarray(fun(x), float).reshape(-1)
    J = np.empty((y.size, x.size))
    for i in range(x.size):
        dx = np.zeros_like(x); dx[i] = h*max(1., abs(x[i]))
        J[:, i] = (fun(x+dx)-fun(x-dx))/(2*dx[i])
    return J
