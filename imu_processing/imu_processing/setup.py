from setuptools import find_packages, setup

package_name = 'imu_processing'

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
    maintainer='emamon',
    maintainer_email='emamon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_processing_raw = imu_processing.imu_processing_raw:main',
            'imu_processing_fake = imu_processing.imu_processing_fake:main',
            'imu_plot = imu_processing.imu_plot:main',
            'imu_calibration_live = imu_processing.imu_calibration_live:main'
            'imu_stationary_record = imu_processing.imu_stationary_record:main'
        ],
    },
)
