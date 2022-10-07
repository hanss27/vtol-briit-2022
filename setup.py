from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['run_server'],
  package_dir={'': 'src/briit/'},
)

setup(**d)
