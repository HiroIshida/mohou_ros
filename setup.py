from setuptools import find_packages, setup

setup_requires = []

install_requires = [
    "numpy",
    "scipy",
    "scikit-robot",
    "mohou>=0.2.3",
    "tunable-filter>=0.0.3"
]

setup(
    name='mohou_ros_utils',
    version='0.0.1',
    description='utility for imitation learning data processing',
    author='Hirokazu Ishida',
    author_email='h-ishida@jsk.imi.i.u-tokyo.ac.jp',
    license='MIT',
    install_requires=install_requires,
    packages=find_packages(exclude=('tests', 'docs'))
)
