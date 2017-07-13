# coding=utf-8
"""
LQR output feedback based control optimization
"""

# noinspection PyUnusedLocal,PyUnusedLocal,PyUnusedLocal
# pylint: disable=invalid-name, no-member
import scipy
import sys
import control
import numpy as np
import ulog_tools as ut
import argparse
import matplotlib.pyplot as plt
import json


# pylint: disable=invalid-name, no-member, too-many-locals, too-many-arguments

def model_to_acc_tf(m):
    acc_tf = m['gain'] * control.tf(*control.pade(m['delay'], 1))  # order 1 approx
    tf_integrator = control.tf((1), (1, 0))
    return acc_tf * tf_integrator

def attitude_loop_design(m, name, dcut_hz, fbcut_hz, est_delay):
    tf_est = control.tf(*control.pade(est_delay, 1)) 
    G_rate_ol = model_to_acc_tf(m)
    K_rate, G_rate_ol, G_rate_comp_cl = ut.lqrofb.pid_design(
        G=G_rate_ol, K_guess=[0.2, 0.2, 0], dcut_hz=dcut_hz, fbcut_hz=fbcut_hz)
    tf_integrator = control.tf([1], [ 1, 0])
    G_ol = G_rate_comp_cl*tf_integrator*tf_est
    K, G_ol, G_comp_cl = ut.lqrofb.pid_design(
        G=G_ol, K_guess=[1], dcut_hz=dcut_hz, fbcut_hz=None,
        use_I=False, use_D=False)
    return {
        'MC_{:s}RATE_P'.format(name): K_rate[0, 0],
        'MC_{:s}RATE_I'.format(name): K_rate[1, 0],
        'MC_{:s}RATE_D'.format(name): K_rate[2, 0],
        'MC_{:s}_P'.format(name): K[0, 0],
    }

def main():
    
    parser = argparse.ArgumentParser()

    file_group = parser.add_mutually_exclusive_group(required=True)
    file_group.add_argument('--url', help="download log from url")
    file_group.add_argument('--file', help="use local file")

    parser.add_argument('--dcut_hz', help="derivative cut freq", default=1000)
    parser.add_argument('--est_delay', help="estimator delay", default=0.15)
    parser.add_argument('--fbcut_hz', help="butterworth cut frequency", default=30)
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
        print('designing controller...',)
    dcut_hz = np.float(args.dcut_hz)
    est_delay = np.float(args.est_delay)
    fbcut_hz = np.float(args.fbcut_hz)
    roll_gains = attitude_loop_design(
        m=res['roll']['model'],
        name='ROLL',
        dcut_hz=dcut_hz,
        fbcut_hz=fbcut_hz,
        est_delay=est_delay
        )
    pitch_gains = attitude_loop_design(
        m=res['pitch']['model'],
        name='PITCH',
        dcut_hz=dcut_hz,
        fbcut_hz=fbcut_hz,
        est_delay=est_delay
        )
    yaw_gains = attitude_loop_design(
        m=res['yaw']['model'],
        name='YAW',
        dcut_hz=dcut_hz,
        fbcut_hz=fbcut_hz,
        est_delay=est_delay
        )
    if args.verbose:
        print('controller designed, writing to file {:s}'.format(args.out))
    out = {}
    out.update(roll_gains)
    out.update(pitch_gains)
    out.update(yaw_gains)
    out.update(res)
    s = json.dumps(out, indent=2, sort_keys=True)
    print(s)
    with open(args.out, 'w') as f:
        f.write(s)
        f.write(json.dumps(res, indent=2))

if __name__ == "__main__":
    main()
