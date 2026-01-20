"""
@file rand_robots_movement.py
@brief Generate training data from random robot configurations
@details Simulates multiple robots with randomly varying motor parameters,
         running identical trajectories with fixed PID gains. Generates
         comprehensive datasets including velocity tracking and trajectory plots.
"""

from functions import *

import control as ct
from matplotlib import pyplot as plt

n = 20 # number of random sets
# ---- Generate n independent sets ----
random_param_sets = [generate_random_motor_params() for _ in range(n)]

print("----------- Generated parameter sets: ----------------\n")
for i, p in enumerate(random_param_sets):
    print(f"Set #{i+1}: {p}\n")
print("------------------------------------------------------")


# ---- Define trajectories ----------
mov_touples_list_list = [
    [
       #(type    ,fw/cw,  lv, ang, r, t)
        ('linear', True,  12,  90, 0, 10),
        ('linear', True,  10,   0, 0, 5),
        ('linear', False, 20,   0, 0, 7),
        ('circular', True, 5, 360, 20, 28)
    ]

]

Ts = 0.01
for i, motor_params in enumerate(random_param_sets):

    print(f"\n===== Running robot set #{i+1} =====")

    # Update system model A, B, C, D for this motor param set
    sys_lin = ct.nlsys(
        updfnc_omniwheel, outfnc_omniwheel, params=motor_params,
        inputs=('u1', 'u2', 'u3'),  outputs=('x', 'y', 'phi', 'dx', 'dy', 'dphi'),
        name='omniwheel_sys', states=('x', 'y', 'phi', 'dx', 'dy', 'dphi')
    )

    Kp = np.array([2.5, 2.4, 2.6])
    Ki = np.array([0.002, 0.001, 0.002])
    Kd = np.array([0.002, 0.001, 0.003])

    # Find the equilibrium point
    X0 = [0, 0, 0, 0, 0, 0]
    eqpt = ct.find_eqpt(sys_lin, X0, 0)
    xeq = eqpt[0]

    # Linearize the system at the equilibrium point
    lin_omni = ct.linearize(sys_lin, xeq, 0)
    sys_z = ct.c2d(lin_omni, Ts, method='zoh')  # Convert to discrete-time system
    A_z, B_z, C_z, D_z = sys_z.A, sys_z.B, sys_z.C, sys_z.D

    sys_lin_z = ct.StateSpace(A_z, B_z, C_z, D_z, Ts)

    for mov_tuples_list in mov_touples_list_list:
        omega_ref, T = generate_omega_ref_trajectory(
            mov_tuples_list=mov_tuples_list,
            Ts = Ts
        )

        kp_str = ', '.join(map(str, Kp))
        ki_str = ', '.join(map(str, Ki))
        kd_str = ', '.join(map(str, Kd))
        set_err, overshoot, osc, y_log, U_pid = run_pid_sim(Kp, Ki, Kd, 0, A_z, B_z, C_z, D_z, T, omega_ref, 
                                                            Ts, save_to_file=True,
                                                            file_name=f"Robot{i+1}_[{kp_str}][{ki_str}][{kd_str}]",
                                                            directory="python/robots_train/")
        
        t_out, y_out = ct.input_output_response(sys_lin_z, T, U_pid.T, X0)

        x_pos = y_out[0]
        y_pos = y_out[1]

        x_pos_world = (x_pos * np.cos((np.pi/4)) - y_pos * np.sin((np.pi/4)))
        y_pos_world = (x_pos * np.sin((np.pi/4)) + y_pos * np.cos((np.pi/4)))

        # Plot results - Velocity tracking
        plt.figure(figsize=(10, 6))
        plt.plot(T, omega_ref[:,0], '--', label='ω_ref1')
        plt.plot(T, y_log[:,3], label='ω1 actual')
        plt.plot(T, omega_ref[:,1], '--', label='ω_ref2')
        plt.plot(T, y_log[:,4], label='ω2 actual')
        plt.plot(T, omega_ref[:,2], '--', label='ω_ref3')
        plt.plot(T, y_log[:,5], label='ω3 actual')
        plt.xlabel('Time [s]')
        plt.ylabel('Wheel Velocities')
        plt.legend()
        plt.title('Incremental PID Velocity Tracking per Wheel')
        plt.grid()
        plt.savefig(f"python/robots_results/Robot{i+1}_velocity_tracking.png", dpi=300, bbox_inches='tight')
        plt.close()
        
        # Plot position outputs (trajectory)
        plt.figure(figsize=(8, 8))
        plt.plot(x_pos_world, y_pos_world, marker="o", label="Robot Path")

        # Ensure equal axis scaling for better spatial interpretation
        plt.gca().set_aspect('equal', adjustable='box')
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Robot Position and Orientation")
        plt.legend()
        plt.grid()
        plt.savefig(f"python/robots_results/Robot{i+1}_trajectory.png", dpi=300, bbox_inches='tight')
        plt.close()