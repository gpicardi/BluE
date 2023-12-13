from setuptools import find_packages, setup

package_name = 'dht11'

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
    maintainer='silvercore',
    maintainer_email='giacomo.picardi1991@gmail.com',
    description='ROS2 node to read and publish data from the temperature and humidity sensor dht11',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'dht11 = dht11.dht11:main'
        ],
    },
)
