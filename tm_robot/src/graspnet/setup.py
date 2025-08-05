from setuptools import find_packages, setup

package_name = 'graspnet'

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
    maintainer='work',
    maintainer_email='work@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_mask = graspnet.yolo_mask:main',
            'grab_detect = graspnet.grab_detect:main',
            'camera2ee = graspnet.camera2ee:main',
            'groundsam = graspnet.groundsam:main',
            'aruco_check = graspnet.aruco_check:main',
            'moveit_grab_test = graspnet.moveit_grab_test:main',
            'camera2base = graspnet.camera2base:main',
            'test_shelf = graspnet.test_shelf:main',
        ],
    },
)
