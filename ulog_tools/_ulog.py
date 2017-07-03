# coding=utf-8
"""
Utilities for ulog
"""

import os
from urllib.request import urlopen

import numpy as np


def download_log(log_url: str, dest_dir: str, name: str = None, overwrite: bool = False):
    """
    Download a log to the desired directory
    """
    if not os.path.isdir(dest_dir):
        os.makedirs(dest_dir)
    if name is None:
        name = log_url.split('=')[1]
    file_name = '{:s}.ulg'.format(name)
    log_path = os.path.join(dest_dir, file_name)
    if overwrite or not os.path.isfile(log_path):
        with open(log_path, 'wb') as f:
            f.write(urlopen(log_url).read())
    return log_path


def sample_frequency(data):
    """Compute sampling frequency"""
    return 1.0e6 / np.diff(data['f_timestamp']).mean()
