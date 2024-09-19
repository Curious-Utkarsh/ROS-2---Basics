from setuptools import find_packages, setup

package_name = 'rover_logic'

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
            "obstacle_avoiding = rover_logic.obstacle_avoiding:main",
            "wall_following = rover_logic.wall_following:main",
            "edge_avoiding = rover_logic.edge_avoiding:main",
            "video_saver = rover_logic.video_saver:main",
            "line_maze_solving = rover_logic.line_maze_solving:main",
            "arucoDetection = rover_logic.arucoDetection:main"
        ],
    },
)
