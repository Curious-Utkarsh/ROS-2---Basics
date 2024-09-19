from setuptools import find_packages, setup

package_name = 'nav2_py'

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
            "nav2_2d_pose_estimator = nav2_py.nav2_2d_pose_estimator:main",
            "nav2_waypoint_follower = nav2_py.nav2_waypoint_follower:main"
        ],
    },
)
