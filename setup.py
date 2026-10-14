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
    entry_points={  # Only for nodes - sits outside the pkg file name
        'console_scripts': [
            'imu_raw_live = imu_processing.imu_raw_live:main',
            'imu_cal_live = imu_processing.imu_cal_live:main',
            'cal_gyro_record = imu_processing.cal_gyro_record:main',
            'cal_mag_record = imu_processing.cal_mag_record:main',
        ],
    },
)
