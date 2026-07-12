from glob import glob

from setuptools import find_packages, setup

package_name = 'um982_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'um982-driver'],
    zip_safe=True,
    maintainer='estudiante',
    maintainer_email='japolo1503@gmail.com',
    description='Velocity and dual-antenna heading publisher for the Unicore UM982 '
                '(ArduSimple simpleRTK3B Compass), via the um982-driver PyPI package.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'um982_heading_node = um982_driver.heading_node:main',
            'configure_um982 = um982_driver.configure_um982:main',
        ],
    },
)
