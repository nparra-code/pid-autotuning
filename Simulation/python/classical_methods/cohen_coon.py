"""  
@file cohen_coon.py
@brief Cohen-Coon PID tuning method implementation
@details Implements the Cohen-Coon tuning rules for P, PI, and PID controllers
         based on process reaction curve parameters
"""

def cohen_coon(type, kp, tm, tau):
  """
  @brief Calculate PID parameters using Cohen-Coon tuning rules
  @details Computes controller gains based on process gain, dead time, and time constant
  
  @param type Controller type: 'P', 'PI', or 'PID'
  @param kp Process gain (steady-state gain)
  @param tm Dead time (time delay) in seconds
  @param tau Time constant in seconds
  
  @return Tuple (Kc, ti, td) where:
          - Kc: Controller gain
          - ti: Integral time constant (0 for P controller)
          - td: Derivative time constant (0 for P and PI controllers)
  """
  if type == 'P':
    return (tau/kp/tm)*(1+tm/3/tau), 0, 0
  elif type == 'PI':
    return (tau/kp/tm)*(0.9+tm/12/tau), (tm*(30+3*tm/tau))/(9+20*tm/tau), 0
  elif type == 'PID':
    return (tau/kp/tm)*((4/3)+(tm/4/tau)), tm*((32+(6*tm/tau))/(13+8*tm/tau)), tm*(4/(11+2*tm/tau))
  else:
    print("Invalid type. Use 'P', 'PI', or 'PID'.")
    return None

# Kc, ti, td = cohen_coon('PID', kp=(10.4)/(20), tm=0.009, tau=0.1780)
# print(f"Kc: {Kc} [%Co/%To], ti: {ti}, td: {td}")