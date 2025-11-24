import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'nav_package'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Uncomment and use if you add launch files later:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mouchmaed Intze',
    maintainer_email='lpxmi2@nottingham.ac.uk',
    description='Navigation client for Minitask 3 autonomous robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_node = nav_package.nav_node:main',
        ],
    },
)

