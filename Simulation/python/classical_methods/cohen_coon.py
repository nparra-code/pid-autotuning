def cohen_coon(type, kp, tm, tau):
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