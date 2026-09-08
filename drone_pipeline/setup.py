from setuptools import find_packages, setup

package_name = 'drone_pipeline'

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
    maintainer='varsha',
    maintainer_email='varshakattimani92@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = drone_pipeline.camera_node:main',
            'detection_node = drone_pipeline.detection_node:main',
            'mock_camera_node = drone_pipeline.mock_camera_node:main',
            'geotag_node = drone_pipeline.geotag_node:main',
            'mission_control_node = drone_pipeline.mission_control_node:main',

        ],
    },
)
