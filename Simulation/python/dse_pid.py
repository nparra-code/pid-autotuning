from functions import *
import control as ct
import matplotlib.pyplot as plt
from itertools import product

# --- Setup system (you already have these defined externally) ---
Ts = 0.01
time = 10
T = np.arange(0, time, Ts)

# Define the nonlinear system
io_sys = ct.nlsys(
    updfnc_omniwheel, outfnc_omniwheel, inputs=('u1', 'u2', 'u3'),
    outputs=('x', 'y', 'phi', 'dx', 'dy', 'dphi'),
    name='omniwheel', states=('x', 'y', 'phi', 'dx', 'dy', 'dphi')
)

# Find the equilibrium point
X0 = [0, 0, 0, 0, 0, 0]
eqpt = ct.find_eqpt(io_sys, X0, 0)
xeq = eqpt[0]

# Linearize the system at the equilibrium point
lin_omni = ct.linearize(io_sys, xeq, 0)
sys_z = ct.c2d(lin_omni, Ts, method='zoh')  # Convert to discrete-time system
A_z, B_z, C_z, D_z = sys_z.A, sys_z.B, sys_z.C, sys_z.D

sys_lin_z = ct.StateSpace(A_z, B_z, C_z, D_z, Ts)

omega_ref, T = generate_omega_ref_trajectory(
    mov_tuples_list=[
        ('linear', True,  1,   0, 0, 2),
        ('linear', True,  1, -90, 0, 2),
        ('linear', False, 1,   0, 0, 2),
        ('linear', False, 1, -90, 0, 2)
    ], Ts=Ts
)

A, B, C, D = A_z, B_z, C_z, D_z

# --- Parameter grids ---
# 20   17  0.03   19   16  0.02   20   17  0.03
Kp_vals1 = np.arange(19.5, 21.5, 0.5)
Ki_vals1 = np.arange(16.0, 18.5, 0.5)
Kd_vals1 = np.arange(0.02, 0.04, 0.01)

Kp_vals2 = Kp_vals1
Ki_vals2 = Ki_vals1
Kd_vals2 = Kd_vals1

Kp_vals3 = Kp_vals1
Ki_vals3 = Ki_vals1
Kd_vals3 = Kd_vals1

results = []

# --- Grid search ---
for Kp_val1, Ki_val1, Kd_val1 in product(Kp_vals1, Ki_vals1, Kd_vals1):
  for Kp_val2, Ki_val2, Kd_val2 in product(Kp_vals2, Ki_vals2, Kd_vals2):
    for Kp_val3, Ki_val3, Kd_val3 in product(Kp_vals3, Ki_vals3, Kd_vals3):
        Kp = np.array([Kp_val1, Kp_val2, Kp_val3])
        Ki = np.array([Ki_val1, Ki_val2, Ki_val3])
        Kd = np.array([Kd_val1, Kd_val2, Kd_val3])

        set_err, overshoot, osc, y_log, U_pid = run_pid_sim(Kp, Ki, Kd, A, B, C, D, T, omega_ref)

        results.append({
            'Kp1': Kp_val1, 'Ki1': Ki_val1, 'Kd1': Kd_val1,
            'Kp2': Kp_val2, 'Ki2': Ki_val2, 'Kd2': Kd_val2,
            'Kp3': Kp_val3, 'Ki3': Ki_val3, 'Kd3': Kd_val3,
            'SetErr': set_err,
            'Overshoot': overshoot,
            'Oscillation': osc
        })

        print(f"Kp1={Kp_val1:.1f}, Ki1={Ki_val1:.1f}, Kd1={Kd_val1:.2f} | "
              f"Kp2={Kp_val2:.1f}, Ki2={Ki_val2:.1f}, Kd2={Kd_val2:.2f} | "
              f"Kp3={Kp_val3:.1f}, Ki3={Ki_val3:.1f}, Kd3={Kd_val3:.2f} | "

              f"Err={set_err:.3f}, Ov={overshoot:.3f}, Osc={osc:.4f}")

# --- Convert results to array for analysis ---
import pandas as pd
df = pd.DataFrame(results)

# Sort by lowest steady-state error, then overshoot
df['score'] = df['SetErr']*3 + df['Overshoot']*2 + df['Oscillation']*0.5
df_sorted = df.sort_values(by='score')

print("\n=== Top 5 PID configurations ===")
print(df_sorted.head())

# --- Optional: visualize top result ---
best = df_sorted.iloc[0]
Kp = np.array([best.Kp1, best.Kp2, best.Kp3])
Ki = np.array([best.Ki1, best.Ki2, best.Ki3])
Kd = np.array([best.Kd1, best.Kd2, best.Kd3])


_, _, _, y_log, _ = run_pid_sim(Kp, Ki, Kd, A, B, C, D, T, omega_ref)

plt.figure(figsize=(12,6))
plt.plot(T, omega_ref[:,0], '--', label='ω_ref1')
plt.plot(T, y_log[:,3], label='ω1')
plt.plot(T, omega_ref[:,1], '--', label='ω_ref2')
plt.plot(T, y_log[:,4], label='ω2')
plt.plot(T, omega_ref[:,2], '--', label='ω_ref3')
plt.plot(T, y_log[:,5], label='ω3')
plt.legend()
plt.grid()
plt.title(f"Best PID: Kp={[best.Kp1, best.Kp2, best.Kp3]}, Ki={[best.Ki1, best.Ki2, best.Ki3]}, Kd={[best.Kd1, best.Kd2, best.Kd3]}\n"
          f"Err={best.SetErr:.3f}, Ov={best.Overshoot:.3f}, Osc={best.Oscillation:.4f}")
plt.xlabel("Time [s]")
plt.ylabel("Wheel velocities")
plt.show()