# Python script to demonstrate phase-envelope with start/end at midpoint and k half-oscillations
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------
# USER PARAMETERS (edit these)
# ---------------------------
mode = 'k'       # 'k' to specify number of half-oscillations, 'n' for number of full oscillations
k = 3            # number of half-oscillations (each half goes from one extreme to the other)
n = 3            # number of full oscillations (ignored if mode == 'k')
f = 0.5          # base frequency of the sinusoidal oscillation in Hz (full cycles per second)
dt = 0.001       # time step for integration (s)

# Example end poses (scalars for easy plotting)
Q1 = -0.5
Q2 = 0.5

# ---------------------------
# Derived quantities
# ---------------------------
omega = 2.0 * np.pi * f               # rad/s for full oscillation
T_half = np.pi / omega                # half-period duration = pi/omega = 1/(2f)
if mode == 'n':
    k_total = int(2 * n)              # convert full oscillations to half-oscillations
else:
    k_total = int(k)

T_total = k_total * T_half            # total time = number of half-periods * half-period duration
t = np.arange(0.0, T_total + dt, dt) # time vector including final instant

# ---------------------------
# Minimum-jerk ramp (0..1 on tau)
# ---------------------------
def min_jerk(tau):
    """Minimum-jerk ramp on tau in [0,1]."""
    return 10.0*tau**3 - 15.0*tau**4 + 6.0*tau**5

# ---------------------------
# Envelope E(t): ramps only on first and last half-periods
# ---------------------------
def envelope_timevec(time_array):
    E = np.ones_like(time_array)
    for i, tt in enumerate(time_array):
        if tt < T_half:
            tau = tt / T_half
            E[i] = min_jerk(tau)
        elif tt > T_total - T_half:
            tau = (T_total - tt) / T_half
            # clamp
            if tau < 0.0:
                tau = 0.0
            E[i] = min_jerk(tau)
        else:
            E[i] = 1.0
    return E

E = envelope_timevec(t)

# ---------------------------
# Integrate phase phi(t) with initial phase so alpha(0)=0.5 (midpoint)
# alpha = 0.5*(1 - cos(phi)) -> need cos(phi)=0 -> phi = pi/2 + m*pi
# choose phi0 = pi/2
# Each half-oscillation corresponds to phi increase of pi, so total phi change = k_total*pi
# ---------------------------
phi = np.zeros_like(t)
phi[0] = 0.5 * np.pi  # start at phi = pi/2 -> alpha = 0.5 (midpoint)
for i in range(1, len(t)):
    phi[i] = phi[i-1] + omega * E[i] * dt

# Optional: ensure we end exactly at phi0 + k_total*pi by small rescale (numerical integration drift)
phi_end_target = phi[0] + k_total * np.pi
phi_scale = (phi_end_target - phi[0]) / (phi[-1] - phi[0])
# apply affine correction so start and end match exactly while preserving shape
phi = phi[0] + (phi - phi[0]) * phi_scale

# ---------------------------
# Alpha and goal trajectory
# ---------------------------
alpha = 0.5 * (1.0 - np.cos(phi))         # bounded in [0,1], starts and ends at 0.5
q_goal = (1.0 - alpha) * Q1 + alpha * Q2  # linear interpolation between Q1 and Q2

# compute time-derivative of alpha for inspection (finite difference)
dalpha_dt = np.gradient(alpha, dt)

# ---------------------------
# PLOTS
# ---------------------------
plt.figure(figsize=(10, 12))

plt.subplot(5,1,1)
plt.plot(t, E)
plt.title("Envelope E(t) — ramp only on first & last half-periods")
plt.ylabel("E(t)")
plt.grid(True)

plt.subplot(5,1,2)
plt.plot(t, phi)
plt.title("Phase φ(t) — integrated with envelope, start at π/2")
plt.ylabel("φ(t) [rad]")
plt.grid(True)

plt.subplot(5,1,3)
plt.plot(t, alpha)
plt.title("Parameter α(t) = 0.5*(1 - cos φ(t)) — starts & ends at 0.5 (midpoint)")
plt.ylabel("α(t)")
plt.ylim([-0.1, 1.1])
plt.grid(True)

plt.subplot(5,1,4)
plt.plot(t, dalpha_dt)
plt.title("Time derivative dα/dt — should be smooth and near-zero at start/end midpoints")
plt.ylabel("dα/dt")
plt.grid(True)

plt.subplot(5,1,5)
plt.plot(t, q_goal)
plt.title("Interpolated q_goal(t) between Q1 and Q2")
plt.xlabel("Time [s]")
plt.ylabel("q_goal")
plt.grid(True)

plt.tight_layout()
plt.show()

# ---------------------------
# PRINT SUMMARY INFO
# ---------------------------
print(f"mode = {mode}, k_total (half-oscillations) = {k_total}, f = {f} Hz")
print(f"T_half = {T_half:.4f} s, T_total = {T_total:.4f} s, omega = {omega:.4f} rad/s")
print(f"phi start = {phi[0]:.4f} rad, phi end target = {phi_end_target:.4f} rad, phi end actual = {phi[-1]:.4f} rad")
