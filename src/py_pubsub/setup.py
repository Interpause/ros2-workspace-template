from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John-Henry Lim',
    maintainer_email='42513874+Interpause@users.noreply.github.com',
    description='Example of minimal pub/sub system using rclpy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publish:main',
            'listener = py_pubsub.subscribe:main',
        ],
    },
)
