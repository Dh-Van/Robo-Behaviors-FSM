from setuptools import find_packages, setup

package_name = 'robo_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_message = robo_behaviors_fsm.send_message:main',
            'drive_circle = robo_behaviors_fsm.drive_circle_node:main',
            'wall_follow = robo_behaviors_fsm.wall_following_node:main',
        ],
    },
)


# .bashrc
'''
build1() {
        cd ~/ros2_ws && colcon build --symlink-install --packages-select robo_behaviors_fsm && source ~/ros2_ws/install/setup.bash
}

run1() {
        ros2 run robo_behaviors_fsm "$1"
}

'''