import os
from setuptools import setup, find_packages


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


setup(name='arm_lightweight_controller',
      version='0.1',
      author='Michal Bednarek',
      description='A high-level framework for programming robotic arms using TCP/IP connection.',
      long_description=read('README.md'),
      author_email='michal.bednarek@put.poznan.pl',
      license='MIT',
      keywords="robotics,arm,manipulator,ur3,ur5,universal,robots",

      # project setup
      packages=find_packages(),
      install_requires=['opencv_python>=3.3.0.10', 'numpy>=1.12.0']
      )

