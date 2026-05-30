from setuptools import find_packages, setup

package_name = 'data_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mudasser',
    maintainer_email='mudasser@todo.todo',
    description='Custom data collection utilities for BCR arm and RX-150 experiments.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collector = data_collector.collector_script:main',
            'point_cloud = data_collector.point_cloud:main',
        ],
    },
)
