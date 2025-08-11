from setuptools import find_packages, setup

package_name = 'suction_point_detection'

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
    maintainer='weng',
    maintainer_email='wengkunduo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ransac = suction_point_detection.ransac:main',
            'test_ransac = suction_point_detection.test_ransac:main',
        ],
    },
)
