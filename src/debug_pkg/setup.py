from setuptools import find_packages, setup

package_name = 'debug_pkg'

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
    maintainer='hhk-laptop',
    maintainer_email='whaihong@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_visualizer_node = debug_pkg.path_visualizer_node:main',	
            'yolov8_visualizer_node = debug_pkg.yolov8_visualizer_node:main',	
            'image_saver_node = debug_pkg.image_saver_node:main',	
			'data_collection_node = debug_pkg.data_collection_node:main',	
			'pose_stamping_node = debug_pkg.pose_stamping_node:main',	
        ],
    },
)
