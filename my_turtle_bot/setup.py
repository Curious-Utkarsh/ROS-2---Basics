from setuptools import find_packages, setup

package_name = 'my_turtle_bot'

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
    maintainer='utk',
    maintainer_email='kutkarsh706@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "spawn_kill_turtle = my_turtle_bot.spawn_kill_turtle:main",
            "turtle_controller = my_turtle_bot.turtle_controller:main",
            "move_turtle_action_server = my_turtle_bot.move_turtle_action_server:main",
            "move_turtlebot = my_turtle_bot.move_turtlebot:main"

        ],
    },
)
