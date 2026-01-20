"""  
@file eval.py
@brief Evaluation script for classical PID tuning methods
@details Compares Ziegler-Nichols and Cohen-Coon tuning methods for
         three-motor omniwheel robot system. Applies tuning rules to
         measured step response data from each motor.
"""

from ziegler_nichols import ziegler_nichols_openloop
from cohen_coon import cohen_coon

Ts = 0.001

Kp_zn, Ki_zn, Kd_zn = ziegler_nichols_openloop('PID', kp=0.394, tm=0.0225, tau=0.0515)
Kp_cc, Ki_cc, Kd_cc = cohen_coon('PID', kp=0.394, tm=0.0225, tau=0.0515)

print("---------------------------------------------")
print("Ziegler-Nichols PID Controller Parameters for Right:")
print(f"Kp: {Kp_zn}, Ki: {Ki_zn:.2f}, Kd: {Kd_zn:.3f}")
print("---------------------------------------------")
print("Cohen-Coon PID Controller Parameters for Right:")
print(f"Kp: {Kp_cc:.2f}, Ki: {Ki_cc:.2f}, Kd: {Kd_cc:.3f}")
print("---------------------------------------------")

Kp_zn, Ki_zn, Kd_zn = ziegler_nichols_openloop('PID', kp=0.452, tm=0.04, tau=0.081)
Kp_cc, Ki_cc, Kd_cc = cohen_coon('PID', kp=0.452, tm=0.04, tau=0.081)

print("---------------------------------------------")
print("Ziegler-Nichols PID Controller Parameters for Left:")
print(f"Kp: {Kp_zn}, Ki: {Ki_zn:.2f}, Kd: {Kd_zn:.3f}")
print("---------------------------------------------")
print("Cohen-Coon PID Controller Parameters for Left:")
print(f"Kp: {Kp_cc:.2f}, Ki: {Ki_cc:.2f}, Kd: {Kd_cc:.3f}")
print("---------------------------------------------")

Kp_zn, Ki_zn, Kd_zn = ziegler_nichols_openloop('PID', kp=0.422, tm=0.051, tau=0.048)
Kp_cc, Ki_cc, Kd_cc = cohen_coon('PID', kp=0.422, tm=0.051, tau=0.048)

print("---------------------------------------------")
print("Ziegler-Nichols PID Controller Parameters for Back:")
print(f"Kp: {Kp_zn}, Ki: {Ki_zn:.2f}, Kd: {Kd_zn:.3f}")
print("---------------------------------------------")
print("Cohen-Coon PID Controller Parameters for Back:")
print(f"Kp: {Kp_cc:.2f}, Ki: {Ki_cc:.2f}, Kd: {Kd_cc:.3f}")
print("---------------------------------------------")