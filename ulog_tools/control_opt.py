# coding=utf-8
"""
LQR output feedback based control optimization
"""

# noinspection PyUnusedLocal,PyUnusedLocal,PyUnusedLocal
# pylint: disable=invalid-name, no-member
import sys
import control
import numpy as np
import ulog_tools as ut
import argparse
import matplotlib.pyplot as plt
import json


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


def pid_design(G, K_guess, d_tc, verbose=False, use_P=True, use_I=True, use_D=True):
    # type: (control.tf, np.array, float, bool, bool, bool, bool) -> (np.array, control.tf, control.tf)
    """
    :param G: transfer function
    :param K_guess: gain matrix guess
    :param d_tc: time constant for derivative
    :param verbose: show debug output
    :param use_P: use p gain in design
    :param use_I: use i gain in design
    :param use_D: use d gain in design
    :return: (K, G_comp, Gc_comp)
        K: gain matrix
        G_comp: open loop compensated plant
        Gc_comp: closed loop compensated plant
    """
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
        print('ogptimizing controller')
        t
    K = lqr_ofb_design(K_guess, ss_open, verbose)
    if verbose:
        print('done')

    # print('K', K)
    # print('H', H)
    G_comp = control.series(G, H * K)
    Gc_comp = control.feedback(G_comp, 1)

    return K, G_comp, Gc_comp

def model_to_acc_tf(m):
    acc_tf = m['gain'] * control.tf(*control.pade(m['delay'], 1))  # order 1 approx
    tf_integrator = control.tf((1), (1, 0))
    return acc_tf * tf_integrator

def attitude_pid_design(model, d_tc=1.0/10, K0=[0.2, 0.2, 0.01, 5]):
    G_rate_ol = model_to_acc_tf(model)
    K_rate, G_cl_rate, G_ol_rate_comp = pid_design(
        G_rate_ol, K0[:3], d_tc)
    tf_integrator = control.tf((1), (1, 0))
    K, G_cl, G_ol_comp = pid_design(
        G_cl_rate*tf_integrator, K0[3:], d_tc,
        use_I=False, use_D=False)
    return np.vstack([K_rate, K])

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()

    file_group = parser.add_mutually_exclusive_group(required=True)
    file_group.add_argument('--url', help="download log from url")
    file_group.add_argument('--file', help="use local file")

    parser.add_argument('--plot', action='store_true', help="enable plotting")
    parser.add_argument('--verbose', action='store_true', help="enable verbose")
    parser.add_argument('out', help="output file")
    args = parser.parse_args()

    if args.url is not None:
        log_file = ut.ulog.download_log(args.url, '/tmp')
    elif args.file is not None:
        log_file = args.file
    else:
        print('must supply url or file')
        parser.print_help()
        sys.exit(-1)

    if args.verbose:
        print('preparing data...',)
    data = ut.sysid.prepare_data(log_file)
    if args.verbose:
        print('data prepared')
        print('identifying attitude dynamics...',)
    res = ut.sysid.attitude_sysid(data, plot=args.plot, verbose=args.verbose)
    if args.plot:
        plt.show()
    if args.verbose:
        print('dynamics identified')
        print(res)
        print('designing controller...',)
    roll_gains = attitude_pid_design(res['roll']['model'], 'ROLL')
    pitch_gains = attitude_pid_design(res['pitch']['model'], 'PITCH')
    yaw_gains = attitude_pid_design(res['yaw']['model'], 'YAW')
    if args.verbose:
        print('controller designed, writing to file {:s}', args.dest, args.verbose)
    out = {}
    out.update(roll_gains)
    out.update(pitch_gains)
    out.update(yaw_gains)
    with open(args.out, 'w') as f:
        f.write(json.dumps(out))
