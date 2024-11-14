from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='jegoh',
    maintainer_email='jeffrey.goh@sit.singaporetech.edu.sg',
    description='ROS2 Topic Activity package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = my_py_pkg.my_first_node:main",
            "robot_news_station = my_py_pkg.robot_news_station:main",
            "smartphone = my_py_pkg.smartphone:main",
            "number_publisher = my_py_pkg.number_publisher:main",
            "number_counter = my_py_pkg.number_counter:main",
            "reset_counter_client = my_py_pkg.reset_counter_client:main",
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
            "add_two_ints_client = my_py_pkg.add_two_ints_client:main",
            "add_two_ints_client_no_oop = my_py_pkg.add_two_ints_client_no_oop:main",
            "hw_status_publisher = my_py_pkg.hw_status_publisher:main",
            "battery_node = my_py_pkg.battery_node:main",
            "led_panel_node = my_py_pkg.led_panel_node:main",
            "spawner = my_py_pkg.spawner:main",
            "controller = my_py_pkg.controller:main",
            "catch_them_all = my_py_pkg.catch_them_all:main"
        ],
    },
)
