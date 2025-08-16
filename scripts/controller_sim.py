# Re-run the refined simulation (state got reset in the environment)
import numpy as np
import matplotlib.pyplot as plt

# --------------------------
# USER TUNABLES
# --------------------------
dt = 0.005                   # simulation time step (s)
k = 20                       # number of half-oscillations for planned envelope window
sync = 1                   # master adaptation gain (0..1)
w_human = 0.7                # human dominance (0..1). 1.0 => human fully drives actual q
# Base gains (will be scaled by sync * envelope)
K_p = 4.0                    # phase adaptation gain
K_omega = 23.8                # frequency adaptation gain
K_a = 1.0                    # amplitude adaptation gain (gradient descent style)
K_c = 1.5                    # center/bias adaptation gain

# Additional refinements / safety
lambda_omega = 0.5           # damping pulling omega toward nominal (rad/s)
initial_robot_frequency = 0.5
omega_nominal = 2*np.pi*initial_robot_frequency  # nominal robot omega (rad/s)
omega_min = 2*np.pi*0.2      # min allowed omega
omega_max = 2*np.pi*1.5      # max allowed omega
A_min, A_max = 0.05, 1.5     # amplitude bounds
C_min, C_max = -1.0, 1.0     # bias bounds
epsilon_b = 1e-3             # small regularizer for tangent normalization

# Error filtering
tau_e = 0.08                 # time constant for low-pass filtering of tangential error (s)
alpha_e = dt / (tau_e + dt)  # discrete low-pass coefficient

# Watchdog (end-of-handshake) parameters
watchdog_window = int(0.5 / dt)   # number of steps to evaluate human activity (0.5s window)
watchdog_move_thresh = 0.01       # RMS movement threshold (rad) -- below this means human stopped
watchdog_error_thresh = 0.01      # filtered tangential error threshold (rad)
watchdog_hold_time = 0.8          # seconds of inactivity required to trigger release
watchdog_counter = 0

# Simulation time derived
omega_robot_initial = omega_nominal
T_half = np.pi / omega_robot_initial
T_total = k * T_half
t = np.arange(0.0, T_total, dt)
n_steps = len(t)

# --------------------------
# Envelope helpers (unchanged)
# --------------------------
def min_jerk(tau):
    return 10.0*tau**3 - 15.0*tau**4 + 6.0*tau**5

def envelope_timevec(time_array, T_half, T_total):
    E = np.ones_like(time_array)
    for i, tt in enumerate(time_array):
        if tt < T_half:
            E[i] = min_jerk(tt / T_half)
        elif tt > T_total - T_half:
            tau = (T_total - tt) / T_half
            E[i] = min_jerk(max(0.0, tau))
    return E

envelope = envelope_timevec(t, T_half, T_total)

# --------------------------
# Human motion (exogenous)
# Slightly ambiguous/irregular periodic motion (frequency & amplitude jitter)
# --------------------------
f_human_base = 0.6
A_human_base = 0.35
C_human_base = 0.0

np.random.seed(1)
human_freq_noise = 0.03 * np.sin(2*np.pi*0.15*t)            # slow freq drift
human_amp_noise = 0.03 * np.sin(2*np.pi*0.2*t + 0.5)       # slow amp modulation
human_phase = np.cumsum(2*np.pi*(f_human_base + human_freq_noise)*dt)
q_human = C_human_base + (A_human_base + human_amp_noise) * np.sin(human_phase)
E = envelope
q_human = E * q_human  # apply envelope to human motion
# add small random jitter (realistic human micro-variations)
q_human += 0.01 * np.random.randn(n_steps)

# --------------------------
# Robot internal state initialization
# --------------------------
phi = np.pi/2.0            # start at midpoint
omega = omega_robot_initial
A = 0.25
C = 0.0

# Histories
q_d_hist = np.zeros(n_steps)
q_actual_hist = np.zeros(n_steps)
omega_hist = np.zeros(n_steps)
A_hist = np.zeros(n_steps)
C_hist = np.zeros(n_steps)
e_q_hist = np.zeros(n_steps)
e_t_filt_hist = np.zeros(n_steps)
watchdog_flag_hist = np.zeros(n_steps, dtype=bool)

# filtered tangential error (initial)
e_t_filt = 0.0

# Watchdog state
handshake_active = True
release_triggered = False
time_since_release = 0.0

# --------------------------
# MAIN SIMULATION LOOP
# --------------------------
for i in range(n_steps):
    tt = t[i]
    E = envelope[i]
    # Commanded (robot) position (note amplitude multiplied by envelope in generator)
    q_d = C - (E * A) * np.cos(phi)

    # Actual shared motion: human dominates with weight w_human
    q_actual = (1.0 - w_human) * q_d + w_human * q_human[i]

    # Error between actual and commanded
    e_q = q_actual - q_d

    # --------------------------
    # REFINEMENT 1: smooth tangent normalization (continuous b)
    # Previously: b = sign(A * sin(phi))
    # Now: use normalized derivative of q_d w.r.t phi with eps regularizer
    # derivative dq_d/dphi = E*A*sin(phi) => unit-like tangent b = sign of derivative but smoothed
    dq_d_dphi = E * A * np.sin(phi)
    b = dq_d_dphi / (np.abs(dq_d_dphi) + epsilon_b)   # smooth approximation to sign(), not discontinuous
    # --------------------------

    # Tangential error (scalar)
    e_t = b * e_q

    # --------------------------
    # REFINEMENT 2: low-pass filter the tangential error before using in adaptation
    # This reduces reaction to high-frequency noise and micro-jitters
    e_t_filt = (1 - alpha_e) * e_t_filt + alpha_e * e_t
    # --------------------------

    # --------------------------
    # WATCHDOG: detect end-of-handshake (human stopped or little interaction)
    # We use two indicators: low human motion RMS over recent window AND small filtered tangential error.
    # If both low for a sustained period -> trigger release.
    if i >= watchdog_window:
        recent_human_window = q_human[i-watchdog_window:i]
        human_rms = np.sqrt(np.mean(np.diff(recent_human_window)**2))
        if (human_rms < watchdog_move_thresh) and (np.abs(e_t_filt) < watchdog_error_thresh):
            watchdog_counter += 1
        else:
            watchdog_counter = 0
        if (watchdog_counter * dt) >= watchdog_hold_time:
            handshake_active = False
            release_triggered = True
    # --------------------------

    # --------------------------
    # ADAPTATION with safety: only adapt while handshake_active, else slowly relax
    current_sync_gain = sync * E if handshake_active else 0.0

    # REFINEMENT 3: damping/leak term on omega pulling toward nominal to prevent drift
    # REFINEMENT 4: use filtered tangential error in the adaptation laws
    phi_dot = omega + current_sync_gain * K_p * e_t_filt
    omega_dot = current_sync_gain * K_omega * e_t_filt - lambda_omega * (omega - omega_nominal)

    # REFINEMENT 5: amplitude & center adapted by gradient-descent style, using raw e_q but scaled by current_sync_gain
    A_dot = - current_sync_gain * K_a * e_q * np.cos(phi)
    C_dot =   current_sync_gain * K_c * e_q

    # Integrate
    phi += phi_dot * dt
    omega += omega_dot * dt
    A += A_dot * dt
    C += C_dot * dt

    # REFINEMENT 6: saturate/bound parameters to safe ranges
    omega = np.clip(omega, omega_min, omega_max)
    A = np.clip(A, A_min, A_max)
    C = np.clip(C, C_min, C_max)

    # If release triggered: gradually ramp out commanded stiffness/trajectory (here we simply reduce envelope via time counter)
    if release_triggered:
        time_since_release += dt
        # simple graceful release: linearly decay envelope influence over 0.6s
        release_decay = max(0.0, 1.0 - time_since_release / 0.6)
        # scale commanded amplitude down for graceful release
        q_d = C - (E * release_decay * A) * np.cos(phi)
        # Optionally, you could set current_sync_gain = 0 here to stop adaptation immediately (we already do handshake_active=False)

    # store histories
    q_d_hist[i] = q_d
    q_actual_hist[i] = q_actual
    omega_hist[i] = omega / (2*np.pi)   # convert to Hz for plotting
    A_hist[i] = A
    C_hist[i] = C
    e_q_hist[i] = e_q
    e_t_filt_hist[i] = e_t_filt
    watchdog_flag_hist[i] = not handshake_active

# --------------------------
# PLOTS
# --------------------------
plt.style.use('seaborn-whitegrid')
fig, axs = plt.subplots(6, 1, figsize=(12, 14), sharex=True)

axs[0].plot(t, q_human, label="Human intended (q_h)", color='green', alpha=0.7)
axs[0].plot(t, q_d_hist, label="Robot commanded (q_d)", color='red', linestyle='--')
axs[0].plot(t, q_actual_hist, label="Shared actual (q)", color='blue')
axs[0].set_ylabel('Position [rad]')
axs[0].legend()

axs[1].plot(t, e_q_hist, label='Raw position error e_q', color='purple')
axs[1].plot(t, e_t_filt_hist, label='Filtered tangential error e_t_filt', color='orange')
axs[1].axhline(y=watchdog_error_thresh, color='gray', linestyle=':', label='watchdog error thresh')
axs[1].set_ylabel('Error [rad]')
axs[1].legend()

axs[2].plot(t, omega_hist, label='Adapted freq (Hz)')
axs[2].axhline(y=(omega_nominal/(2*np.pi)), color='gray', linestyle=':', label='omega_nominal (Hz)')
axs2 = axs[2].twinx()
axs2.plot(t, envelope, color='gray', alpha=0.4, linestyle='--', label='envelope')
axs[2].set_ylabel('Frequency [Hz]')
axs[2].legend(loc='upper left')
axs2.legend(loc='upper right')

axs[3].plot(t, A_hist, label='Adapted Amplitude A')
axs[3].axhline(y=A_human_base, color='green', linestyle=':', label='human base amp')
axs[3].set_ylabel('Amplitude')
axs[3].legend()

axs[4].plot(t, C_hist, label='Adapted Bias C')
axs[4].axhline(y=C_human_base, color='green', linestyle=':', label='human base bias')
axs[4].set_ylabel('Bias (center)')
axs[4].legend()

axs[5].plot(t, watchdog_flag_hist.astype(float), label='handshake_active (0=active,1=released)', color='black')
axs[5].set_ylabel('Watchdog')
axs[5].set_xlabel('Time [s]')
axs[5].legend()

plt.tight_layout()
plt.show()

print("Release triggered:", release_triggered)
