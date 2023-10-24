import math

# Algorithm 1: Direct attack trajectory planning.
def interceptor_trajectory(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, φt, dt, mpc_horizon_length):
    x, y, z = xint, yint, ztgt
    i = 0
    vel = vint
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

# Example usage:
xtgt, ytgt, ztgt = 10, 10, 10
xint, yint = 0, 0
vint = 1
vmax = 5
amax = 0.5
φt = math.radians(45)  # Assuming φt is given in degrees and converted to radians
dt = 1
mpc_horizon_length = 10

trajectory = interceptor_trajectory(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, φt, dt, mpc_horizon_length)
print(trajectory)

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

# Example usage:
xtgt = [1, 2, 3, 4, 5]
ytgt = [1, 2, 3, 4, 5]
ztgt = [0, 0, 0, 0, 0]  # Assuming a simple target trajectory for illustration
xint, yint = 0, 0
vint = 1
vmax = 5
amax = 0.5
dt = 1

sample_num = closest_interception_sample(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt)
print(sample_num)

# Algorithm 3: Determining the first sample to be used as a reference for target following.
def find_reference_sample(xtgt, ytgt, ztgt, xp, yp, zp, dist):
    i = -1
    tmpdist = dist3D(xtgt, ytgt, ztgt, xp[i], yp[i], zp[i])
    while tmpdist < dist:
        i -= 1
        tmpdist += dist3D(xp[i+1], yp[i+1], zp[i+1], xp[i], yp[i], zp[i])
    return i

# Example usage:
xtgt, ytgt, ztgt = 5, 5, 5
xp = [0, 1, 2, 3, 4, 5]
yp = [0, 1, 2, 3, 4, 5]
zp = [0, 0, 0, 0, 0, 0]
dist = 7

m = find_reference_sample(xtgt, ytgt, ztgt, xp, yp, zp, dist)
print(m)

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

# Example usage:
xe = [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
ye = [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
ze = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
n = -5
p = 5
des_vel = 1
dt = 1
xint, yint, zint = 0, 0, 0

trajectory = reference_trajectory(xe, ye, ze, n, p, des_vel, dt, xint, yint, zint)
print(trajectory)

# Helper functions: 2D and 3D distances
def distance2D(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def dist3D(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

# You can now call any of the four main functions above in this integrated script.
