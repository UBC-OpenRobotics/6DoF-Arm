from setuptools import find_packages, setup

package_name = 'bcr_arm_common'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Shared RX-150 kinematics and collision model used by bcr_arm_rx150 and data_collector.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
