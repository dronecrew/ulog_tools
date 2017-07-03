# coding=utf-8
# coding=utf-8
"""
LQR output feedback based control optimization
"""

# noinspection PyUnusedLocal,PyUnusedLocal,PyUnusedLocal
# pylint: disable=invalid-name, no-member
import control
import numpy as np
import scipy.optimize


# noinspection PyPep8Naming
def K_from_design_vector(x):
    """Turn design vector into gain matrix"""
    K = np.zeros((9, 3))
    K[0, 0] = x[0]
    K[1, 0] = x[1]
    K[2, 0] = x[2]
    K[3, 1] = x[3]
    K[4, 1] = x[4]
    K[5, 1] = x[5]
    K[6, 2] = x[6]
    K[7, 2] = x[7]
    K[8, 2] = x[8]
    return K


# noinspection PyPep8Naming
def design_vector_from_K(K):
    """Create design vector from gain matrix"""
    return [K[0, 0], K[1, 0], K[2, 0], K[3, 1], K[4, 1], K[5, 1], K[6, 2], K[7, 2], K[8, 2]]


# noinspection PyPep8Naming,PyUnusedLocal
def dlqr_ofb_con(x, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Constraint for LQR output feedback optimization.
    This asserts that all eigenvalues are negative so that
    the system is stable.
    @x design vector for gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return constraint
    """
    # pylint: disable=unused-argument
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    K = K_from_design_vector(x)
    A_c = A - B * K * C
    # print(np.abs(np.real(np.linalg.eig(A_c)[0])) )
    return 1 - np.abs(np.real(np.linalg.eig(A_c)[0]))


# noinspection PyPep8Naming
def dlqr_ofb_cost(x, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Cost for LQR output feedback optimization.
    @x design vector for gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return cost
    """
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    K = K_from_design_vector(x)
    A_c = A - B * K * C
    Q_c = C.T * K.T * R * K * C + Q
    P = scipy.linalg.solve_discrete_lyapunov(A_c.T, -Q_c)
    J = np.trace(P * X)
    return J


# noinspection PyPep8Naming
def dlqr_ofb_jac(x, R, Q, X, ss_o):
    # type: (np.array, np.array, np.array, np.array, control.ss) -> np.array
    """
    Jacobian for LQR Output feedback optimization.
    TODO: might be an error here, doesn't not help optim
    """
    K = K_from_design_vector(x)
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B * K * C
    Q_c = C.T * K.T * R * K * C + Q
    P = scipy.linalg.solve_discrete_lyapunov(A_c.T, -Q_c)
    S = scipy.linalg.solve_discrete_lyapunov(A_c, -X)
    J = 2 * (R * K * C * S * C.T - B.T * P * S * C.T)
    J = np.array(J)[:, 0]
    return J


# noinspection PyPep8Naming
def dlqr_ofb_design(x0, ss_o, verbose=False, input_cost=1e-3, error_cost=1):
    """
    LQR output feedback controller design.
    @K_guess initial stabilizing gains
    @ss_o open loop state space system
    @return gain matrix
    """
    n_x = ss_o.A.shape[0]
    n_u = ss_o.B.shape[1]
    # n_y = ss_o.C.shape[0]
    R = input_cost * np.eye(n_u)
    Q = error_cost * np.eye(n_x)
    X = 1e-3 * np.eye(n_x)

    constraints = [
        {'type': 'ineq',
         'fun': dlqr_ofb_con,
         'args': (R, Q, X, ss_o),
         }
    ]

    res = scipy.optimize.minimize(
        fun=dlqr_ofb_cost,
        # method='SLSQP',
        # method='NELDER-MEAD',
        args=(R, Q, X, ss_o),
        x0=x0,
        # jac=dlqr_ofb_jac,
        bounds=len(x0) * [[0, 100]],
        constraints=constraints
    )
    opt_x = res['x']
    K = K_from_design_vector(opt_x)

    if verbose:
        print(res)
        print('cost', dlqr_ofb_cost(opt_x, R, Q, X, ss_o))
        # print('jac', dlqr_ofb_jac(opt_x, R, Q, X, ss_o))
        print('constraint', dlqr_ofb_con(opt_x, R, Q, X, ss_o))
    if not res['success']:
        print('optimization failed')

    return K
