#!/usr/bin/env python
# coding=utf-8
"""Ulog process tools

This module add processing modules for ulog log files.

"""

from __future__ import print_function

import sys

from setuptools import setup, find_packages

import versioneer

if sys.version_info < (3, 0):
    sys.exit('Sorry, Python < 3.0 is not supported')

DOCLINES = __doc__.split("\n")

CLASSIFIERS = """\
Development Status :: 1 - Planning
Intended Audience :: Science/Research
Intended Audience :: Developers
License :: OSI Approved :: BSD License
Programming Language :: Python
Programming Language :: Python :: 3
Programming Language :: Other
Topic :: Software Development
Topic :: Scientific/Engineering :: Artificial Intelligence
Topic :: Scientific/Engineering :: Mathematics
Topic :: Scientific/Engineering :: Physics
Operating System :: Microsoft :: Windows
Operating System :: POSIX
Operating System :: Unix
Operating System :: MacOS
"""

# pylint: disable=invalid-name


setup(
    name='ulog_tools',
    maintainer="James Goppert",
    maintainer_email="james.goppert@gmail.com",
    description=DOCLINES[0],
    long_description="\n".join(DOCLINES[2:]),
    url='https://github.com/dronecrew/ulog_tools',
    author='James Goppert',
    author_email='james.goppert@gmail.com',
    download_url='https://github.com/dronecrew/ulog_tools',
    license='BSD 3-Clause',
    classifiers=[_f for _f in CLASSIFIERS.split('\n') if _f],
    platforms=["Windows", "Linux", "Solaris", "Mac OS-X", "Unix"],
    install_requires=[
        'scipy', 'numpy', 'control', 'matplotlib', 'slycot',
        'pandas >= 0.19.2', 'transforms3d', 'pyulog', 'sympy'],
    tests_require=['nose', 'coverage'],
    test_suite='nose.collector',
    entry_points={
        'console_scripts': ['ulog_tools_pid_design=ulog_tools.control_opt:main'],
    },
    packages=find_packages(),
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
)
