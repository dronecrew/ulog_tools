"""
Analyze a PX4 log to perform sysid and control design.
"""
from __future__ import print_function

from collections import OrderedDict

import control
import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize
import scipy.signal


# pylint: disable=invalid-name, no-member, too-many-locals, too-many-arguments

# noinspection PyUnusedLocal,PyUnusedLocal,PyUnusedLocal
def lqr_ofb_con(K, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Constraint for LQR output feedback optimization.
    This asserts that all eigenvalues are negative so that
    the system is stable.
    @K gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return constraint
    """
    # pylint: disable=unused-argument
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B * K * C
    return -np.real(np.linalg.eig(A_c)[0])


def lqr_ofb_cost(K, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Cost for LQR output feedback optimization.
    @K gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return cost
    """
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B * K * C
    Q_c = C.T * K.T * R * K * C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    J = np.trace(P * X)
    return J


def lqr_ofb_jac(K, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Jacobian for LQR Output feedback optimization.
    TODO: might be an error here, doesn't not help optim
    """
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B * K * C
    Q_c = C.T * K.T * R * K * C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    S = scipy.linalg.solve_lyapunov(A_c, -X)
    J = 2 * (R * K * C * S * C.T - B.T * P * S * C.T)
    J = np.array(J)[:, 0]
    return J


def lqr_ofb_design(K_guess, ss_o, verbose=False):
    """
    LQR output feedback controller design.
    @K_guess initial stabilizing gains
    @ss_o open loop state space system
    @return gain matrix
    """
    n_x = ss_o.A.shape[0]
    n_u = ss_o.B.shape[1]
    R = 1e-6 * np.eye(n_u)
    Q = np.eye(n_x)
    X = 1e-3 * np.eye(n_x)

    constraints = [
        {'type': 'ineq',
         'fun': lqr_ofb_con,
         'args': (R, Q, X, ss_o),
         }
    ]

    res = scipy.optimize.minimize(
        fun=lqr_ofb_cost,
        method='SLSQP',
        args=(R, Q, X, ss_o),
        x0=K_guess,
        # jac=lqr_ofb_jac,
        bounds=len(K_guess) * [[1e-6, 100]],
        constraints=constraints
    )
    K = np.matrix(res['x'])

    if verbose:
        print(res)
    if not res['success']:
        print('cost', lqr_ofb_cost(K, R, Q, X, ss_o))
        print('jac', lqr_ofb_jac(K, R, Q, X, ss_o))
        print('constraint', lqr_ofb_con(K, R, Q, X, ss_o))
        raise RuntimeError('optimization failed')

    return K.T


def pid_design(G, K_guess, dcut_hz, fbcut_hz=None, verbose=False, use_P=True, use_I=True, use_D=True):
    # type: (control.tf, np.array, float, bool, bool, bool, bool) -> (np.array, control.tf, control.tf)
    """
    :param G: transfer function
    :param K_guess: gain matrix guess
    :param dcut_hz: derivative low pass filter cut frequency
    :param fbcut_hz: feedback cut frequency for 2nd order butterworth, disabled if None
    :param verbose: show debug output
    :param use_P: use p gain in design
    :param use_I: use i gain in design
    :param use_D: use d gain in design
    :return: (K, G_comp, Gc_comp)
        K: gain matrix
        G_comp: open loop compensated plant
        Gc_comp: closed loop compensated plant
    """
    d_tc = 1.0/ (2*np.pi*dcut_hz)

    # compensator transfer function
    H = []
    if use_P:
        H += [control.tf(1, 1)]
    if use_I:
        H += [control.tf((1), (1, 0))]
    if use_D:
        H += [control.tf((1, 0), (d_tc, 1))]
    H = np.array([H]).T
    H_num = [[H[i][j].num[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H_den = [[H[i][j].den[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H = control.tf(H_num, H_den)

    # print('G', G)
    # print('H', H)

    ss_open = control.tf2ss(G * H)

    if verbose:
        print('optimizing controller')
    K = lqr_ofb_design(K_guess, ss_open, verbose)
    if verbose:
        print('done')

    # print('K', K)
    # print('H', H)
    G_comp = control.series(G, H * K)

    # 2nd order butterworth filter for angular rate feedback
    if fbcut_hz is not None:
        wc = fbcut_hz*2*np.pi
        H_butter = control.tf((1), (1/(wc**2), 1.4142/wc, 1))
        Gc_comp = control.feedback(G_comp, H_butter)
    else:
        Gc_comp = control.feedback(G_comp, 1)

    return K, G_comp, Gc_comp


def plot_loops(name, G_ol, G_cl):
    # type: (str, control.tf, control.tf) -> None
    """
    Plot loops
    :param name: Name of axis
    :param G_ol: open loop transfer function
    :param G_cl: closed loop transfer function
    """
    plt.figure()
    plt.plot(*control.step_response(G_cl, np.linspace(0, 1, 1000)))
    plt.title(name + ' step response')
    plt.grid()

    plt.figure()
    control.bode(G_ol)
    print('margins', control.margin(G_ol))
    plt.subplot(211)
    plt.title(name + ' open loop bode plot')

    plt.figure()
    control.rlocus(G_ol, np.logspace(-2, 0, 1000))
    for pole in G_cl.pole():
        plt.plot(np.real(pole), np.imag(pole), 'rs')
    plt.title(name + ' root locus')
    plt.grid()

