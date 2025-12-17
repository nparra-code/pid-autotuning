def cohen_coon(type, k, theta, tau):
  if type == 'P':
    return (tau/k/theta)*(1+theta/3/tau), 0, 0
  elif type == 'PI':
    return (tau/k/theta)*(0.9+theta/12/tau), (theta*(30+3*theta/tau))/(9+20*theta/tau), 0
  elif type == 'PID':
    return (tau/k/theta)*(16+3*theta/tau)/12, (theta*(32+6*theta/tau))/(13+8*theta/tau), 4*theta/(11+2*theta/tau)
  else:
    print("Invalid type. Use 'P', 'PI', or 'PID'.")
    return None

# Kc, ti, td = cohen_coon('PID', k=(10.4)/(20), theta=0.009, tau=0.1780)
# print(f"Kc: {Kc} [%Co/%To], ti: {ti}, td: {td}")