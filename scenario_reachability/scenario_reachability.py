from numpy.random import default_rng
import numpy as np
from numpy import sin, cos, pi
from scipy.integrate import odeint, solve_ivp
import matplotlib.pyplot as plt

def solve(initial_state, times, integrate_func, derivative_func):
    """
    Solves the initial-value problem of the first order ODEs
    :param initial_state: initial state
    :param times: a sequence of time points for which to solve
    :param integrate_func: calculates the next state
    :param derivative_func: computes derivatives of each state component
    :return:
    """
    dt = times[1] - times[0]
    states = [initial_state]
    for step, t in enumerate(times):
        states.append(integrate_func(states[-1], step, t, dt, derivative_func))
    return np.array(states)

def integrate_rk4(state, step, t, dt, dydx_func):
    """
    Fourth-order Runge-Kutta method.
    Source: https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
    :param step:
    :param state:
    :param t:
    :param dt:
    :param dydx_func:
    :return:
    """
    k1 = dydx_func(state, step, t, dt)
    k2 = dydx_func([v + d * dt / 2 for v, d in zip(state, k1)], step, t, dt)
    k3 = dydx_func([v + d * dt / 2 for v, d in zip(state, k2)], step, t, dt)
    k4 = dydx_func([v + d * dt for v, d in zip(state, k3)], step, t, dt)

    return [v + (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6 for v, k1_, k2_, k3_, k4_ in zip(state, k1, k2, k3, k4)]

def derivate(state, time):
    dth, th, dphi, phi = state

    r = 0.25
    l = 1.0
    M = 0.25
    m = 0.3
    g = 9.8
    I = 0.5 * M * r

    _dphi = (m * l * r * dth ** 2 * sin(th) - m * g * r * sin(th) * cos(th)) / (m * r ** 2 * sin(th) ** 2 + I)
    _dth = (g * sin(th) - r * _dphi * cos(th)) / l

    return [_dth, dth, _dphi, dphi]

def make_sample():
    rng = default_rng()
    ru = rng.uniform
    times = np.linspace(0, 10, 500)

    state0 = np.array([ru(-1.7, 1.7), ru(-0.8, 0.8), 
                ru(0.3, 2.0),  ru(-1.0, 1.0)])

    solution = odeint(derivate, state0, times)

    return solution[-1]


def main():

    ndata = 1000
    data = []

    for n in range(ndata):
        s = make_sample()
        data.append(s)
    
    data = np.array(data)

    data_new = []
    for n in range(len(data)):
        if data[n, 3] >= 0:
            data_new.append([data[n, 1], data[n, 3]])
        else:
            data_new.append([data[n, 1], 0])

        # data_new.append([data[n, 1], data[n, 3]])
    data_new = np.array(data_new)
  
    plotting = 0

    Alo = [-100, -100]
    Ahi = [100, 100]
    Ax = [Alo[0], Ahi[0]]
    Ay = [Alo[1], Ahi[1]]
    Awidth = np.subtract(Ahi, Alo)

    xy_num = 0
    ell = 20
    Axspan = np.linspace(Ax[0],Ax[1],ell+1)
    Ayspan = np.linspace(Ay[0],Ay[1],ell+1)

    hits = np.zeros([ell,ell])
 
    plt.subplot(1,2,1)
    plt.box(True)
    plt.grid(True)
    plt.plot(data_new[:, 0], data_new[:,1], 'blue', marker='.', linestyle='None')

    support_constraints = np.zeros([ell,ell])

    for i in range(len(data_new)):
        gridind = []
        gridind = np.ceil(((data_new[i,:] - Alo) / Awidth) * ell)

        xl = Axspan[np.intc(gridind[0])-1]
        xu = Axspan[np.intc(gridind[0])]
        yl = Ayspan[np.intc(gridind[1])-1]
        yu = Ayspan[np.intc(gridind[1])]
            
        plotting = 1

        if not(np.any(hits[np.intc(gridind[0]),  np.intc(gridind[1])])) and plotting:
            plt.subplot(1,2,1)
            plt.fill([xl,xl,xu,xu,xl],[yl,yu,yu,yl,yl], 'blue', alpha=0.2)
            xy_num += 1
            support_constraints[np.intc(gridind[0]), np.intc(gridind[1])] = 1
        
        else:
            support_constraints[np.intc(gridind[0]), np.intc(gridind[1])] = 0

        hits[np.intc(gridind[0]), np.intc(gridind[1])] = 1


    xy = 0
    plot_sc = np.zeros([ell,ell])
    for k in range(ell):
        for h in range(ell):
            if np.sum(support_constraints[k, h]) == 1: 
                if not(np.any(plot_sc[k,h])):
                    xl = Axspan[k-1]
                    xu = Axspan[k]
                    yl = Ayspan[h-1]
                    yu = Ayspan[h]

                    plt.subplot(1,2,1)
                    plt.fill([xl,xl,xu,xu,xl],[yl,yu,yu,yl,yl], 'orange', alpha=0.5)
                    xy += 1
                

                plot_sc[k,h] = 1
        

    if plotting:
        plt.subplot(1,2,1)
        plt.xticks(Axspan)
        plt.yticks(Ayspan)
        plt.xlabel('$\Theta$')
        plt.ylabel('$\phi$')
        plt.show()

    print(xy_num, xy)




    
if __name__ == '__main__':
    main()