from setuptools import find_packages, setup
from glob import glob

package_name = 'eod_av_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    scripts=['scripts/record_dataset.sh'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='estudiante',
    maintainer_email='japolo1503@gmail.com',
    description='Orquestacion de bring-up del stack de sensores EOD-AV: un '
                'launch por sensor + un master (all_sensors.launch.py) sin '
                'launches duplicados.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # TF ajustable en caliente por parametros (calibracion en vivo).
            'tf_tuner = eod_av_launch.tf_tuner:main',
            # Bayer -> RGB para visualizacion (RViz no demosaica Bayer).
            'debayer_node = eod_av_launch.debayer_node:main',
        ],
    },
)
