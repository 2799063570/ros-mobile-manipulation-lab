from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
setup(**generate_distutils_setup(packages=['aubo_thesis_experiments'], package_dir={'': 'src'}))
