## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# For these changes to take affect run: catkin_make and source .bashrc
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# build scripts
setup_args = generate_distutils_setup(
packages=['estimator', 'stateMachine', 'navigator', 'controller', 'sensorInterface', 'configs'],
package_dir={'': 'scripts'},
)
setup(**setup_args)



