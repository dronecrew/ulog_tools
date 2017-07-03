# coding=utf-8
"""
Utilities for python control
"""

from typing import List, Dict, Union

import control
import matplotlib.pyplot as plt
import numpy as np
import sympy


# pylint: disable=no-member, invalid-name


# noinspection PyPep8Naming
def bode(tf: Union[control.TransferFunction, control.StateSpace],
         deg: bool=True, dB: bool=True, Hz: bool=True, Plot: bool=True):
    """Computer bode plot, to fix issues with bode from python control"""
    mag, phase, w = control.bode(tf, deg=deg, dB=dB, Hz=Hz, Plot=False)
    if Plot:
        plt.subplot(211)
        plt.semilogx(w, mag)
        plt.grid(which='both')
        plt.ylabel('magnitude (dB)')

        plt.subplot(212)
        plt.semilogx(w, phase)
        plt.xlabel('Hz')
        plt.ylabel('phase (deg)')
        plt.grid(which='both')
    return mag, phase, w


# noinspection PyPep8Naming
def sympy_to_tf(G, data):
    """convert a discrete transfer function in sympy to a contorl.tf"""
    z = sympy.symbols('z')
    Gs = G.subs(data)
    try:
        num = np.array(sympy.Poly(sympy.numer(Gs), z).all_coeffs(), dtype=float)
        den = np.array(sympy.Poly(sympy.denom(Gs), z).all_coeffs(), dtype=float)
    except Exception:
        raise TypeError('failed to convert expression to float polynomials: ', Gs)
    return control.tf(num, den, 1.0 / data['f_s'])


def delay_gain_model_to_tf(model: Dict):
    """Convert a delay and gain model to a contro.tf"""
    d = model['sample_delay']
    k = model['gain']
    den = np.zeros(d + 1)
    den[0] = 1
    num = k
    return control.tf(num, den, 1.0 / model['f_s'])


def tf_hstack(tf_list: List[control.TransferFunction]):
    """
    Stack so that you add more outputs
    """
    num = []
    den = []
    n_tf = len(tf_list)
    tf0 = tf_list[0]
    n_inputs = tf0.inputs
    n_outputs = tf0.outputs
    for i in range(n_outputs * n_tf):
        num.append([])
        den.append([])
        for j in range(n_inputs):
            num[i].append([])
            den[i].append([])
    for i_tf, tf in enumerate(tf_list):
        assert tf.inputs == n_inputs
        assert tf.outputs == n_outputs
        for i in range(tf.outputs):
            for j in range(tf.inputs):
                k = i + i_tf * n_outputs
                # print('i', i, 'j', j, 'k', k)
                # print('tf shape', tf.inputs, tf.outputs)
                # print('num shape', len(num[0]), len(num))
                num[k][j] = tf.num[i][j]
                den[k][j] = tf.den[i][j]
    return control.tf(num, den)


def tf_vstack(tf_list: List[control.TransferFunction]):
    """
    Stack so that you add more inputs
    """
    num = []
    den = []
    n_tf = len(tf_list)
    tf0 = tf_list[0]
    n_inputs = tf0.inputs
    n_outputs = tf0.outputs
    for i in range(n_outputs):
        num.append([])
        den.append([])
        for j in range(n_inputs * n_tf):
            num[i].append([])
            den[i].append([])
    for i_tf, tf in enumerate(tf_list):
        assert tf.inputs == n_inputs
        assert tf.outputs == n_outputs
        for i in range(tf.outputs):
            for j in range(tf.inputs):
                k = j + i_tf * n_inputs
                # print('i', i, 'j', j, 'k', k)
                # print('tf shape', tf.inputs, tf.outputs)
                # print('num shape', len(num[0]), len(num))
                num[i][k] = tf.num[i][j]
                den[i][k] = tf.den[i][j]
    return control.tf(num, den, tf0.dt)


def tf_dstack(tf_list: List[control.TransferFunction]):
    """
    Diagonal stack so that you have more inputs and outputs
    """
    num = []
    den = []
    n_tf = len(tf_list)
    tf0 = tf_list[0]
    n_inputs = tf0.inputs
    n_outputs = tf0.outputs
    for i in range(n_outputs * n_tf):
        num.append([])
        den.append([])
        for j in range(n_inputs * n_tf):
            num[i].append([0])
            den[i].append([1])
    for i_tf, tf in enumerate(tf_list):
        assert tf.inputs == n_inputs
        assert tf.outputs == n_outputs
        for i in range(tf.outputs):
            for j in range(tf.inputs):
                k = i + i_tf * n_outputs
                l = j + i_tf * n_inputs
                # print('i', i, 'j', j, 'k', k)
                # print('tf shape', tf.inputs, tf.outputs)
                # print('num shape', len(num[0]), len(num))
                num[k][l] = tf.num[i][j]
                den[k][l] = tf.den[i][j]
    return control.tf(num, den, tf0.dt)
