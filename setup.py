from setuptools import setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'freenove_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        (os.path.join('share', package_name), glob('params/*')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seuberte',
    maintainer_email='seuberte@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface2driver = freenove_base.interface2driver:main',
            'state_publisher = freenove_base.state_publisher:main'
        ],
    },
)
