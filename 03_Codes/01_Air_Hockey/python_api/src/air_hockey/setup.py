from setuptools import setup, find_packages
setup(
name='air_hockey',
version='0.1.2',
author='Tan Çağatay Acar',
author_email='acartancagatay@hotmail.com',
description='Python package for ROS integrated air-hockey table designed at METU ROMER',
packages=find_packages(),
classifiers=[
'Programming Language :: Python :: 3',
'License :: OSI Approved :: MIT License',
'Operating System :: OS Independent',
],
python_requires='>=3.8',
)