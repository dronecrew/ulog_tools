# coding=utf-8
"""
Tests for the control_utils module.
"""

import os
import unittest

import control
import matplotlib.pyplot as plt
import pyulog
import sympy

import ulog_tools as ut

TEST_LOG_URL = "https://logs.px4.io/download?log=0467b169-aec0-44d0-bbd0-a42cce863acf"
TEST_DIR = os.path.realpath(os.path.dirname(__file__))
TEST_TMP_DIR = os.path.join(TEST_DIR, 'tmp')


class Test(unittest.TestCase):
    def setUp(self):
        test_log_path = ut.ulog.download_log(TEST_LOG_URL, TEST_TMP_DIR)
        self.log = pyulog.ULog(test_log_path)

    def test_download_log(self):
        ut.ulog.download_log(TEST_LOG_URL, TEST_TMP_DIR)

    def test_bode(self):
        tf = control.tf([1, 2], [1, 2, 3])
        ut.control.bode(tf, Plot=True)
        plt.savefig(os.path.join(TEST_TMP_DIR, 'bode.png'))

    def test_tf_hstask(self):
        tf = control.tf([1, 2], [1, 2, 3])
        tf_new = ut.control.tf_hstack([tf, tf, tf])
        self.assertEqual(tf_new.inputs, 1)
        self.assertEqual(tf_new.outputs, 3)

    def test_tf_vstask(self):
        tf = control.tf([1, 2], [1, 2, 3])
        tf_new = ut.control.tf_vstack([tf, tf, tf])
        self.assertEqual(tf_new.inputs, 3)
        self.assertEqual(tf_new.outputs, 1)

    def test_tf_dstask(self):
        tf = control.tf([1, 2], [1, 2, 3])
        tf_new = ut.control.tf_dstack([tf, tf, tf])
        self.assertEqual(tf_new.inputs, 3)
        self.assertEqual(tf_new.outputs, 3)

    def test_sympy_to_tf(self):
        z, f_s = sympy.symbols('z, f_s')
        ut.control.sympy_to_tf(f_s*z / (z**2 + 1), {'f_s': 1.0})
