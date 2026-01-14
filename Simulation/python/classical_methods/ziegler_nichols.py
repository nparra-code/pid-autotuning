# Ziegler-Nichols PID Tuning Rules

## Comments in Doxygen

## @brief Ziegler-Nichols open-loop tuning method
#
# @param type Controller type (P, PI, PID)
# @param L Dead time
# @param T Time constant
# @return Tuple of (Kc, ti, td) for the specified controller type
def ziegler_nichols_openloop(type, kp, tau, tm):
  if type == 'P':
    return tm/tau, 10e20, 0
  elif type == 'PI':
    return 0.9*tm/tau, tau/0.3, 0
  elif type == 'PID':
    return (1.2*tau/(kp*tm), 2*tau/(kp*tm)), 2*tm, 0.5*tm 

def ziegler_nichols_closedloop(type, Kcr, Pcr):
  if type == 'P':
    return 0.5*Kcr, 10e20, 0
  elif type == 'PI':
    return 0.45*Kcr, Pcr/1.2, 0
  elif type == 'PID':
    return 0.6*Kcr, 0.5*Pcr, 0.125*Pcr

# Kc, ti, td = ziegler_nichols_openloop('PID', L=0.009, T=0.1780)
# print(f"Kc: {Kc} [%Co/%To], ti: {ti}, td: {td}")