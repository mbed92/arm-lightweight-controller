import os
import sys

from setuptools import setup, find_packages

CURRENT_PYTHON = sys.version_info[:2]
REQUIRED_PYTHON = (2, 7)

if CURRENT_PYTHON > REQUIRED_PYTHON:
    sys.stderr.write("========================== \n"
                     "Unsupported Python version \n"
                     "Your version: {} \n"
                     "Required: {} \n"
                     "==========================\n".format(CURRENT_PYTHON, REQUIRED_PYTHON))
    sys.exit(1)


def read(fname):
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


setup(name='robot_controller',
      version='0.3',
      author='Michal Bednarek',
      description='A high-level framework for programming robotic arms using TCP/IP connection.',
      long_description=read('README.md'),
      long_description_content_type='text/markdown',
      author_email='michal.bednarek@put.poznan.pl',
      license='MIT',
      keywords="robotics,arm,manipulator,ur3,ur5,universal,robots",

      # project setup
      packages=find_packages(),
      install_requires=['opencv_python==4.2.0.32', 'numpy>=1.12.0'],
      py_modules=['robot_controller']
      )
