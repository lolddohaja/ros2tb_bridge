from setuptools import find_packages, setup

package_name = 'ros2tb_bridge'

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
    maintainer='zeta',
    maintainer_email='zeta@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synchronized_subscriber = ros2tb_bridge.synchronized_subscriber:main',
            'threaded_synchronized_subscriber = ros2tb_bridge.threaded_synchronized_subscriber:main',
            'async_synchronized_subscriber = ros2tb_bridge.async_synchronized_subscriber:main',
            'thingsboard_bridge = ros2tb_bridge.mqtt_synchronized_subscriber:main',
        ],
    },
)
