# coding=utf-8
"""
Log based system ID
"""
from typing import Dict, List

import control
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyulog
import scipy.optimize
import scipy.signal as sig

import ulog_tools as ut


# pylint: disable=no-member, invalid-name


def ulog_to_dict(log: pyulog.ULog) -> Dict:
    """Convert ulog to a dict"""
    res = {}
    for topic in log.data_list:
        types = {}
        for field in topic.field_data:
            types[field.field_name] = field.type_str
        index = pd.TimedeltaIndex(data=np.array(topic.data['timestamp']), unit='us')
        cols = []
        data = []
        for k in sorted(topic.data.keys()):
            cols.append('f_{:s}'.format(k.replace('[', '_').replace(']', '')))
            data.append(topic.data[k])
        data = np.array(data).T
        df = pd.DataFrame(data, index=index, columns=cols)
        res['t_{:s}_{:d}'.format(topic.name, topic.multi_id)] = df
    return res


def series_diff(series: pd.Series, order: int=1) -> pd.Series:
    """Derivative of a series"""
    dx = np.gradient(series.data, order)
    dt = np.gradient(series.index, order)
    dxdt = np.zeros(dx.shape)
    for i, dt_i in enumerate(dt):
        if dt_i <= 0:
            dxdt[i] = 0
        else:
            dxdt[i] = dx[i] / dt[i]
    return pd.Series(data=dxdt, index=series.index)


class IirFilter(object):
    """IIR filter"""

    # pylint: disable=too-few-public-methods

    def __init__(self, wp: List[float], ws: List[float],
                 gpass: float, gstop: float, fs: float, ftype: str):
        # pylint: disable=too-many-arguments
        nyq = fs / 2.0
        self.wp_d = np.array(wp) / nyq
        self.ws_d = np.array(ws) / nyq
        self.gpass = gpass
        self.gstop = gstop
        self.fs = fs
        self.ftype = ftype

    def apply(self, x: pd.Series) -> pd.Series:
        """Apply the filter to x"""
        # noinspection PyTupleAssignmentBalance,PyTypeChecker
        b, a = sig.iirdesign(wp=self.wp_d, ws=self.ws_d,
                             gpass=self.gpass, gstop=self.gstop,
                             ftype=self.ftype)

        data = sig.lfilter(b, a, x)
        return pd.Series(data=data, index=x.index)


def find_delay_gain(y: np.array, u: np.array, fs: float, name: str='', plot: bool=False):
    # type: (np.array, np.array, float, str, bool) -> Dict
    """Find delay and gain to match y to u"""
    # pylint: disable=too-many-locals
    costs = []
    delays = []
    gains = []
    # noinspection PyTypeChecker
    assert np.all(y.index == u.index)

    # noinspection PyShadowingNames
    def f_cost(y, u, i, k):
        """Signal sum sq error cost"""
        e = y - k * u.shift(i).bfill()
        return e.dot(e)

    max_delay = 0.3
    n_delay = int(max_delay*fs)
    for i in range(n_delay):
        # pylint: disable=cell-var-from-loop
        # noinspection PyTypeChecker
        res = scipy.optimize.minimize(
                lambda k_: f_cost(y, u, i, k_), x0=50,
                method='SLSQP',
                bounds=[(0, 1000)],
                )
        k = res['x'][0]
        # if it failed for anything but a line search, print result
        if res['success'] != True and res['status'] != 8:
            print(res)
        costs.append(f_cost(y, u, i, k))
        gains.append(k)
        delays.append(i)

    # noinspection PyTypeChecker
    sample_delay = delays[np.argmin(costs)]
    # noinspection PyTypeChecker
    gain = gains[np.argmin(costs)]

    if plot:
        plt.figure(figsize=(10, 5))
        costs = np.array(costs)
        delays = np.array(delays)
        plt.plot(1e3 * delays / fs, costs, '-')
        plt.xlabel('delay [msec]')
        plt.ylabel('sum sq error')
        plt.title('{:s} delay'.format(name))
        plt.grid()

        plt.figure(figsize=(10, 5))
        (gain * u.shift(sample_delay)).plot(
            label='{:s} acc cmd filtered'.format(name), alpha=0.5)
        y.plot(label='{:s} acc filtered'.format(name), alpha=0.5)
        plt.legend()
        plt.grid()
        plt.ylabel('$rad / s^2$')

    e = y - gain * u.shift(sample_delay).bfill()
    np.var(e)
    fit = 1 - np.var(e) / np.var(y)
    return {
        'sample_delay': sample_delay,
        'delay': sample_delay / fs,
        'gain': gain,
        'fit': fit,
        'f_s': fs,
    }


def prepare_data(filename: str):
    """Combine gyro and actuator data, and resample"""
    log = ulog_to_dict(pyulog.ULog(filename))
    combined = pd.merge_asof(
        log['t_actuator_controls_0_0'],
        log['t_sensor_combined_0'], on='f_timestamp')
    combined.index = pd.TimedeltaIndex(
        data=combined['f_timestamp'] -
             combined['f_timestamp'][0], unit='us')
    sample_period_micros = int(np.ceil(
        1e6 / np.floor(ut.ulog.sample_frequency(combined))))
    combined = combined.resample('{:d} us'.format(
        sample_period_micros)).ffill().bfill()
    # pylint: disable=redefined-variable-type
    combined.index = pd.Float64Index(data=np.array(
        combined.index.values, dtype=np.float) / 1.0e9, name='time, s')
    return combined


def fit_best(y: np.array, u: np.array, fs: float, name: str, window: int=5,
        plot: bool=False, verbose: bool=False, log_stop: float=1):
    # type: (np.array, np.array, float, str, int, bool) -> Dict
    """Return the model for the best window in the log
    log_stop: 0-1 , percentage, 1 means process the entire log
    """
    i = 0
    best = {
        't_start': None,
        't_end': None,
        'model': None,
    }
    tf = y.index[-1]
    n_windows = np.floor(tf/window)
    if verbose:
        print('finding best fit window for {:s}'.format(name))
    while i < int(n_windows*log_stop):
        if verbose and i%10 == 0:
            print('{:d}%'.format(int(100*i/n_windows)))
        t0 = i * window
        t1 = (i + 1) * window
        res = find_delay_gain(
            y=y[t0:t1],
            u=u[t0:t1],
            fs=fs,
            name=name, plot=False)
        # pylint: disable=unsubscriptable-object
        if (best['model'] is None) or (res['fit'] > best['model']['fit']):
            best['model'] = res
            best['t_start'] = t0
            best['t_end'] = t1
        i += 1
    if verbose:
        print('100%')
    t0 = best['t_start']
    t1 = best['t_end']
    find_delay_gain(
        y=y[t0:t1],
        u=u[t0:t1],
        fs=fs,
        name=name, plot=plot)
    return best


def attitude_sysid(data: Dict, plot: bool=False, verbose: bool=False):
    """Perform attitude system ID"""
    fs = ut.ulog.sample_frequency(data)
    if fs < 200:
        raise ValueError("sampling frequency too low: {:5.0f} Hz, needs to be > 200 Hz".format(fs))
    # noinspection PyTypeChecker
    bandpass = IirFilter(wp=[1, 5], ws=[0.5, 20], gpass=0.01,
                         gstop=40, fs=fs, ftype='ellip')
    # noinspection PyDictCreation
    res = {}
    res['roll'] = fit_best(
        y=series_diff(bandpass.apply(data['f_gyro_rad_0'])),
        u=bandpass.apply(data['f_control_0']),
        fs=fs,
        name='roll', plot=plot, verbose=verbose)

    res['pitch'] = fit_best(
        y=series_diff(bandpass.apply(data['f_gyro_rad_1'])),
        u=bandpass.apply(data['f_control_1']),
        fs=fs,
        name='pitch', plot=plot, verbose=verbose)

    res['yaw'] = fit_best(
        y=series_diff(bandpass.apply(data['f_gyro_rad_2'])),
        u=bandpass.apply(data['f_control_2']),
        fs=fs,
        name='yaw', plot=plot, verbose=verbose)
    return res


def delay_gain_model_to_tf(model: Dict):
    """Convert a delay and gain model to a contro.tf"""
    d = model['sample_delay']
    k = model['gain']
    den = np.zeros(d + 1)
    den[0] = 1
    num = k
    return control.tf(num, den, 1.0 / model['f_s'])
