"""  
@file movements.py
@brief Movement primitives for omniwheel robot control
@details Provides functions to convert between linear and angular velocities,
         and generate movement commands for linear and circular paths
"""

from math import sqrt, sin, cos, pi
N = 1
R = 1
DELTA = pi/4

def lin_to_ang_vel(x_velocity, y_velocity, wheel):
  """
  @brief Convert linear velocities to angular wheel velocity
  @details Applies inverse kinematics to compute individual wheel velocities
           from desired robot body velocities
  
  @param x_velocity Desired x-direction velocity (m/s)
  @param y_velocity Desired y-direction velocity (m/s)
  @param wheel Wheel number (1=right, 2=left, 3=back)
  
  @return Angular velocity of specified wheel (rad/s)
  """
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
  """
  @brief Generate velocity command for linear motion
  @details Computes x and y velocity components for straight-line motion at specified angle
  
  @param f Direction flag: True for forward, False for reverse
  @param lin_vel Linear velocity magnitude (m/s)
  @param angle Direction angle in degrees (0=north, 90=east)
  
  @return Tuple (vx, vy) of velocity components (m/s)
  """
  if f == True:
    return -lin_vel * sin(angle * pi / 180.0), -lin_vel * cos(angle * pi / 180.0)
  else:
    return lin_vel * sin(angle * pi / 180.0), lin_vel * cos(angle * pi / 180.0)

def circ_move(cw, lin_velocity, angle, radius, t):
    """
    @brief Generate velocity components for circular motion
    @details Computes instantaneous velocity for following a circular arc
    
    @param cw Direction flag: True for clockwise, False for counter-clockwise
    @param lin_velocity Tangential velocity (linear speed along the circle) (m/s)
    @param angle Total angle to traverse in degrees
    @param radius Radius of the circular path (m)
    @param t Current time instant (s)
    
    @return Tuple (vx, vy) of velocity components in x and y directions (m/s)
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