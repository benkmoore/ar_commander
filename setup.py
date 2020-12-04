## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# For these changes to take affect run: catkin_make and source .bashrc
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# build scripts
setup_args = generate_distutils_setup(
packages=['estimator', 'stateMachine', 'navigator', 'controller', 'sensorInterface', 'utils'],
package_dir={'': 'scripts'},
)
setup(**setup_args)

# build configs
setup_args = generate_distutils_setup(
packages=['sim_params', 'robot_v1', 'hardware_params'],
package_dir={'': 'configs'},
)
setup(**setup_args)



