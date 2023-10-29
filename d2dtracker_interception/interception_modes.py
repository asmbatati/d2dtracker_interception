import math

# Algorithm 1: Direct attack trajectory planning.
def interceptor_trajectory(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt, mpc_horizon_length):
    x, y, z = xint, yint, ztgt
    i = 0
    vel = vint
    φt = yaw_to_tgt(xtgt, ytgt, xint, yint)
    traj = []

    while i < mpc_horizon_length:
        vel += amax * dt
        if vel > vmax:
            vel = vmax
        
        x += vel * math.cos(φt)
        y += vel * math.sin(φt)
        
        traj.append((x, y, z))
        i += 1

    return traj

# Algorithm 2: Point of interception estimation in case of a dynamic target.
def closest_interception_sample(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt):
    x, y, z = xint, yint, ztgt[0]
    i = 0
    vel = vint
    dtarget = distance2D(x, y, xtgt[0], ytgt[0])
    dflown = 0
    num_samples = len(xtgt)
    
    while i < num_samples:
        vel += amax * dt
        if vel > vmax:
            vel = vmax
        dflown += vel * dt
        dtarget = distance2D(x, y, xtgt[i], ytgt[i])
        if dflown > dtarget:
            return i
        i += 1

    return -1

# Algorithm 3: Determining the first sample to be used as a reference for target following.
def find_reference_sample(xtgt, ytgt, ztgt, xp, yp, zp, dist):
    i = -1
    tmpdist = dist3D(xtgt, ytgt, ztgt, xp[i], yp[i], zp[i])
    while tmpdist < dist:
        i -= 1
        tmpdist += dist3D(xp[i+1], yp[i+1], zp[i+1], xp[i], yp[i], zp[i])
    return i

# Algorithm 4: Head on collision trajectory planning.
def reference_trajectory(xe, ye, ze, n, p, des_vel, dt, xint, yint, zint):
    traj = []
    dist = dist3D(xe[p], ye[p], ze[p], xint, yint, zint)
    m = p
    for i in range(p-1, -1, -1):
        tmpdist = dist3D(xe[i], ye[i], ze[i], xint, yint, zint)
        if tmpdist < dist:
            dist = tmpdist
            m = i
    traj.append((xe[m], ye[m], ze[m]))
    tmpdist = 0
    for i in range(m-1, n, -1):
        tmpdist += dist3D(xe[i], ye[i], ze[i], xe[i-1], ye[i-1], ze[i-1])
        if tmpdist > dt * des_vel:
            traj.append((xe[i], ye[i], ze[i]))
            tmpdist -= dt * des_vel
    return traj

# Helper functions: 2D and 3D distances
def distance2D(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def dist3D(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

def yaw_to_tgt(x1, y1, x2, y2):
    return math.atan2((y2 - y1),(x2 - x1))
