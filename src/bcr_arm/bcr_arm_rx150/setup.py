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
            'launch/rx150_pick_place.launch.py',
            'launch/rx150_pick_place_sim.launch.py',
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/rx150_gripper_depth_camera.urdf.xacro',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/rx150_obstacles.world',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/rx150_dls_sim_stack.rviz',
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
            'mission_keyboard = bcr_arm_rx150.mission_keyboard:main',
            'rx150_dls_ik_executor = bcr_arm_rx150.rx150_dls_ik_executor:main',
            'rx150_gripper_controller = bcr_arm_rx150.rx150_gripper_controller:main',
            'rx150_joint_waypoint_executor = bcr_arm_rx150.rx150_joint_waypoint_executor:main',
            'rx150_named_pose = bcr_arm_rx150.rx150_named_pose:main',
            'rx150_path_waypoint_executor = bcr_arm_rx150.rx150_path_waypoint_executor:main',
            'rx150_pick_place_orchestrator = bcr_arm_rx150.rx150_pick_place_orchestrator:main',
            'rx150_point_cloud_path_planner = bcr_arm_rx150.rx150_point_cloud_path_planner:main',
            'sweep_placeholder = bcr_arm_rx150.sweep_placeholder:main',
            'vision_placeholder = bcr_arm_rx150.vision_placeholder:main',
            'rx150_smoke_test = bcr_arm_rx150.rx150_smoke_test:main',
            'rx150_target_test_suite = bcr_arm_rx150.rx150_target_test_suite:main',
        ],
    },
)
