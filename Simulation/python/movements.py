from math import sqrt, sin, cos, pi
N = 1
R = 1
DELTA = pi/4

def lin_to_ang_vel(x_velocity, y_velocity, wheel):
  scale = N / R

  cos_d = cos(DELTA)
  sin_d = sin(DELTA)

  # wb = 0, so ignore third column
  # wheel is a constant expression
  if wheel == 1: # right wheel
    return scale * ( - sin_d * x_velocity - cos_d * y_velocity )
  elif wheel == 2: # left wheel
    return scale * ( -sin_d * x_velocity + cos_d * y_velocity )
  elif wheel == 3: # back wheel
    return scale * ( x_velocity )

def lin_move(f, lin_vel, angle):
  if f == True:
    return -lin_vel * sin(angle * pi / 180.0), -lin_vel * cos(angle * pi / 180.0)
  else:
    return lin_vel * sin(angle * pi / 180.0), lin_vel * cos(angle * pi / 180.0)

def circ_move(cw, lin_velocity, angle, radius, t):
    """
    Generate velocity components for circular motion.
    
    Args:
        cw: True for clockwise, False for counter-clockwise
        lin_velocity: tangential velocity (linear speed along the circle)
        angle: total angle to traverse in degrees
        radius: radius of the circular path
        t: current time instant
    
    Returns:
        (vx, vy): velocity components in x and y directions
    """
    # Calculate time needed to complete the desired angle
    time_to_complete = (angle / 360.0) * 2 * pi * radius / lin_velocity
    
    if t < time_to_complete:
        # Current angle traversed (in radians)
        theta = (lin_velocity / radius) * t
        
        # Velocity components for circular motion
        # For counter-clockwise motion (standard mathematical convention)
        vx = -lin_velocity * sin(theta)
        vy = lin_velocity * cos(theta)
        
        # Flip direction for clockwise motion
        if cw:
            return vx, -vy
        else:
            return vx, vy
    else:
        # Motion complete, return zero velocity
        return 0.0, 0.0

x_v, y_v = lin_move(True, 3, 7)
print(x_v, y_v)
r_w_v = lin_to_ang_vel(x_v, y_v, 1)
print('Right wheel velocity:', r_w_v)
l_w_v = lin_to_ang_vel(x_v, y_v, 2)
print('Left wheel velocity:', l_w_v)
b_w_v = lin_to_ang_vel(x_v, y_v, 3)
print('Back wheel velocity:', b_w_v)