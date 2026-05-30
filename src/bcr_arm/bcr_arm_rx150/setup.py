from setuptools import find_packages, setup

package_name = 'bcr_arm_rx150'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rx150_control.launch.py',
            'launch/rx150_dls_sim_stack.launch.py',
            'launch/rx150_gz_classic.launch.py',
            'launch/rx150_dls_stack.launch.py',
            'launch/rx150_moveit_interface.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mudasser',
    maintainer_email='mudasser@todo.todo',
    description='RX-150 bringup and test utilities for the BCR arm workspace.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rx150_dls_ik_executor = bcr_arm_rx150.rx150_dls_ik_executor:main',
            'rx150_named_pose = bcr_arm_rx150.rx150_named_pose:main',
            'rx150_smoke_test = bcr_arm_rx150.rx150_smoke_test:main',
            'rx150_target_test_suite = bcr_arm_rx150.rx150_target_test_suite:main',
        ],
    },
)
