from setuptools import find_packages
from setuptools import setup

package_name = 'csv_pkg'

setup(
    name=package_name,
    version='0.7.1',
    packages=find_packages(exclude=['test']),
    install_requires=['launch', 'setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml', 'launch/launchExperiment.py']),
    ],
    zip_safe=True,
    author='Antonio Marino',
    author_email='marino.antonio96@gmail.com ',
    maintainer='Antonio Marino',
    maintainer_email='marino.antonio96@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python nodes for logging in ULISSE Project'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pars = csv_pkg.pars:main',
            'log_status_context = csv_pkg.log_status_context:main',
            'log_control_context = csv_pkg.log_control_context:main',
            'log_nav_filter_data = csv_pkg.log_nav_filter_data:main',
            'log_llc_gps = csv_pkg.log_llc_gps:main',
            'log_llc_imu = csv_pkg.log_llc_imu:main',
            'log_llc_magnetometer = csv_pkg.log_llc_magnetometer:main',
            'log_llc_compass = csv_pkg.log_llc_compass:main',
            'log_generic = csv_pkg.log_generic:main',
            'stop_experiments = csv_pkg.stop_experiments:main',
            'make_circle = csv_pkg.make_circle:main',
            'make_ellipse = csv_pkg.make_ellipse:main',
        ],
    },
)