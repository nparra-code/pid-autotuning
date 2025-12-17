from ziegler_nichols import ziegler_nichols_openloop
from cohen_coon import cohen_coon
from s_to_z import s_z_domain

Ts = 0.001

dead_time          = 0.009  # Example dead time
transient_response = 0.1780  # Example transient response time

d_input  = ( 20 - 0 )  # Input delta
d_output = ( 10.4 - 0 )  # Output delta
kappa    = d_output / d_input

Kp_zn, Ki_zn, Kd_zn = ziegler_nichols_openloop('PID', L=dead_time, T=transient_response)
Kp_cc, Ki_cc, Kd_cc = cohen_coon('PID', k=kappa, theta=dead_time, tau=transient_response)

print("---------------------------------------------")
print("Ziegler-Nichols PID Controller Parameters:")
print(f"Kp: {Kp_zn:.2f}, Ki: {Ki_zn:.2f}, Kd: {Kd_zn:.3f}")
print("---------------------------------------------")
print("Cohen-Coon PID Controller Parameters:")
print(f"Kp: {Kp_cc:.2f}, Ki: {Ki_cc:.2f}, Kd: {Kd_cc:.3f}")
print("---------------------------------------------")

K_zn_z = s_z_domain(Kp_zn, Ki_zn, Kd_zn, Ts)
K_cc_z = s_z_domain(Kp_cc, Ki_cc, Kd_cc, Ts)

print("---------------------------------------------")
print("Ziegler-Nichols PID Controller Parameters (Z-domain):")
print(f"Kp: {K_zn_z['Kp_disc']:.2f}, Ki: {K_zn_z['Ki_z']:.2f}, Kd: {K_zn_z['Kd_z']:.3f}")
print("---------------------------------------------")
print("Cohen-Coon PID Controller Parameters (Z-domain):")
print(f"Kp: {K_cc_z['Kp_disc']:.2f}, Ki: {K_cc_z['Ki_z']:.2f}, Kd: {K_cc_z['Kd_z']:.3f}")
print("---------------------------------------------")