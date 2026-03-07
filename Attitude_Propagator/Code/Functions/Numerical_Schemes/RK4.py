import numpy as np

def RK4(odefun, y0, t_final, dt, sim_input=None):
    """
    RK4 solves an ODE system using the 4th-order Runge-Kutta method.

    This function integrates a system of ordinary differential equations (ODEs)
    using the Runge-Kutta 4th-order method (RK4) with given initial conditions.
    It returns the solution at each time step for the specified time range.

    INPUTS:
        odefun   : callable
                   Function defining the ODE: dydt = odefun(t, y, sim_input)
        y0       : ndarray
                   Initial condition vector
        t_final  : float
                   Final time for integration
        dt       : float
                   Time step size
        sim_input: object, optional
                   Additional input arguments for the ODE function (necessary in some cases).

    OUTPUTS:
        t : ndarray
            Vector of time points where the solution is computed
        Y : ndarray
            Matrix of solutions (each row corresponds to the state at a time point)
    """

    # Time vector
    t = np.arange(0, t_final + dt, dt)
    N = len(t)

    # Solution array
    Y = np.zeros((N, len(y0)))
    Y[0, :] = y0

    # Runge-Kutta integration 
    for i in range(N - 1):
        ti = t[i]
        yi = Y[i, :]

        # RK4 coefficients
        k1 = dt * odefun(ti, yi, sim_input)
        k2 = dt * odefun(ti + dt / 2, yi + k1 / 2, sim_input)
        k3 = dt * odefun(ti + dt / 2, yi + k2 / 2, sim_input)
        k4 = dt * odefun(ti + dt, yi + k3, sim_input)

        # Next state 
        Y[i + 1, :] = yi + (k1 + 2 * k2 + 2 * k3 + k4) / 6

    return t, Y





