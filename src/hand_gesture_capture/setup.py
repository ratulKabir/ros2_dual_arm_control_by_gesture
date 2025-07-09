from setuptools import find_packages, setup

package_name = 'hand_gesture_capture'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'custom_msgs'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='raihankabirratul@gmail.com',
    description='Hand tracking and gesture capture with ROS 2',
    license='MIT',  # or update with your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_publisher_node = hand_gesture_capture.hand_publisher_node:main',
        ],
    },
)
