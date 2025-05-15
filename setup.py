from setuptools import find_packages, setup

package_name = 'mb_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds',  ['worlds/worlds/world.wbt']),
        ('share/' + package_name + '/worlds',  ['worlds/proto/Astra.proto']),
        ('share/' + package_name + '/worlds',  ['worlds/proto/Rosbot.proto']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rviz.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rtab.launch.py']),
        ('share/' + package_name + '/launch', ['launch/nav2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/lidar_slam.launch.py']),
        ('share/' + package_name + '/resource', ['resource/mb.urdf']),
        ('share/' + package_name + '/config', ['config/ros2control.yaml']), 
        ('share/' + package_name + '/config', ['config/rviz.rviz']), 
        ('share/' + package_name + '/config', ['config/ekf.yaml']), 
        ('share/' + package_name + '/config', ['config/amcl_params.yaml']), 
        ('share/' + package_name + '/config', ['config/indoor_map_vslam.yaml']), 
        ('share/' + package_name + '/config', ['config/indoor_map_vslam.pgm']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viswa',
    maintainer_email='vss.viswatejabottu@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_driver = mb_description.diff_driver:main',
            'image_viwer = mb_description.image_show:main',
            'data_collector = mb_description.data_collector:main'
        ],
    },
)
