import interception_modes

def engage(xtgt, ytgt, ztgt, xint, yint, vint, vmax, amax, dt, mpc_horizon_length, xp, yp, zp):
    if distance2D(xint, yint,xp, yp) < distance2D(xint, yint,xtgt, ytgt):
        reference_trajectory()
    else:
        find_reference_sample()
        if vtgt <= vint:
            interceptor_trajectory()
