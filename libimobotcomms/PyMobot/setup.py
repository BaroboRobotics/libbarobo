#!/usr/bin/env python

from distutils.core import setup,Extension
from distutils.command.build_py import build_py

dist = setup(name='PyMobot',
    version='0.1',
    description='Mobot Control Python Library',
    author='David Ko',
    author_email='david@barobo.com',
    url='http://www.barobo.com',
    packages=['barobo'],
    ext_modules=[Extension('barobo._mobot', 
      ['barobo/mobot.i'],
      swig_opts=['-c++', '-I../'],
      include_dirs=['../', '../BaroboConfigFile', '../BaroboConfigFile/mxml-2.7'],
      define_macros=[('NONRELEASE',1)],
      extra_compile_args=['-fpermissive'],
      library_dirs=['../', '../BaroboConfigFile', '../BaroboConfigFile/mxml-2.7'],
      libraries=['mobotStatic', 'baroboconfigfile', 'mxml', 'pthread', 'bluetooth', 'rt'],
      )],
    )

build_py = build_py(dist)
build_py.ensure_finalized()
build_py.run()
