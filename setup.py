from setuptools import find_packages, setup

package_name = 'robo_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_circle = robo_behaviors_fsm.drive_circle_node:main',
            'wall_follow = robo_behaviors_fsm.wall_following_node:main',
            'state_machine = robo_behaviors_fsm.state_machine_node:main',
            'person_following = robo_behaviors_fsm.person_following_node:main',
            'test = robo_behaviors_fsm.laser_test_node:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch.py']),
    ],
)


# .bashrc
'''build1() {
        cd ~/ros2_ws && colcon build --symlink-install --packages-select robo_behaviors_fsm && source ~/ros2_ws/install/setup.bash
}

run1() {
        ros2 run robo_behaviors_fsm "$1"
}
        ros2 run robo_behaviors_fsm "$1"
}
testing

ros2 topic pub --once /wall_found std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /wall_end std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /found_person std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"

'''
