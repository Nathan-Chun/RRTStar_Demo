from setuptools import find_packages, setup

package_name = 'point_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/move_point.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/point.urdf']),
        ('share/' + package_name + '/urdf', ['worlds/obstacle_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nchun@usc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_cbf = point_demo.ik_cbf:main',
            'rrt_star = point_demo.rrt_star:main',
        ],
    },
)
