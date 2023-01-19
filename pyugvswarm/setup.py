#!/usr/bin/env python
from setuptools import find_packages
from setuptools import setup

setup(
    name='pyugvswarm',
    version='1.0',
    packages=find_packages('pyugvswarm'),

    description='python wrapper for ugv swarm control',
    url='https://gitee.com/shu-peixuan/pyugvswarm',

    author='Peixuan Shu',
    author_email='shupeixuan@qq.com',
    license='GPLv3',

    classifiers=[
        'Development Status :: 2 - Pre-ALpha',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Topic :: System :: Hardware :: Hardware Drivers',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 3'
    ],

    keywords='ugv swarm python APIs',

    install_requires=[
        'numpy',
        'matplotlib'
    ],

    # $ pip install -e .
)
