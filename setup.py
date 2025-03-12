from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['controller',
              'robot'],
    package_dir={'controller': 'controller',
                 'robot': 'robot'},
)
setup(**setup_args)