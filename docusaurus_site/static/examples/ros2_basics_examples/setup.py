from setuptools import setup

package_name = 'ros2_basics_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Examples for the ROS 2 Basics tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = ros2_basics_examples.simple_publisher:main',
            'simple_subscriber = ros2_basics_examples.simple_subscriber:main',
            'simple_service_server = ros2_basics_examples.simple_service_server:main',
            'simple_service_client = ros2_basics_examples.simple_service_client:main',
        ],
    },
)
