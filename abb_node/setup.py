#!/usr/bin/env python

from distutils.core import setup

setup(name='open_abb',
      version='0.1',
      description='Control ABB robots remotely with ROS, Python, or C++',
      author='Michael Dawson-Haggerty',
      maintainer='Juan G. Victores, David Estevez',
      url='https://github.com/roboticslab-uc3m/open_abb',
      package_dir={'':'packages/abb_communications'},
      py_modules=['abb'],
     )


