from setuptools import find_packages, setup

package_name = 'pub_sub'

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
    maintainer='ashay',
    maintainer_email='ashaywakode@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_node = pub_sub.sub_node:main',
            'sensor_msg_pub_node = pub_sub.sensor_msg_pub_node:main',
            'geometry_msg_pub_node = pub_sub.geometry_msg_pub_node:main',
            'std_msg_pub_node = pub_sub.std_msg_pub_node:main',
        ],
    },
)
