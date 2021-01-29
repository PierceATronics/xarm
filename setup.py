from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['xarm_servo_controller'],
    package_dir={'': 'xArmServoController'},
)

setup(**setup_args)
