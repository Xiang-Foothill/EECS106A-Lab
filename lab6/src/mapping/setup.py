from setuptools import find_packages, setup
from glob import glob
package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/mapping/launch', glob('launch/*.launch.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee106a-abz',
    maintainer_email='liu.yx@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mapping_node=mapping.mapping_node:main',
            'occupancy_grid=mapping.occupancy_grid_2d:main'
        ],
    },
)
