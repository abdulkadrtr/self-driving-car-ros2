from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'config'),glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abdulkadir.ture@std.yildiz.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_controller = autonomous_controller.autonomous_controller:main',
            'neural_network = autonomous_controller.neural_network:main',
        ],
    },
)

