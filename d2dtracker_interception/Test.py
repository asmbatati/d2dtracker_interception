import interception

# Alg1 Example usage:
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

# Alg2 Example usage:
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

# Alg3 Example usage:
xtgt, ytgt, ztgt = 5, 5, 5
xp = [0, 1, 2, 3, 4, 5]
yp = [0, 1, 2, 3, 4, 5]
zp = [0, 0, 0, 0, 0, 0]
dist = 7

m = find_reference_sample(xtgt, ytgt, ztgt, xp, yp, zp, dist)
print(m)

# Alg4 Example usage:
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