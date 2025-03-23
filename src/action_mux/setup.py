from setuptools import find_packages, setup

package_name = 'action_mux'

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
            'goal_publisher = action_mux.goal_publisher:main',
            'action_client = action_mux.action_client:main',
            'action_server = action_mux.action_server:main',
        ],
    },
)
