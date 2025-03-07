from setuptools import setup

package_name = 'imu_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_sensor.launch.py']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimjeongmin',
    maintainer_email='kimjeongmin@example.com',
    description='IMU Sensor Plotter Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'posvel = imu_plotter.posvel:main',
                'imu_publ = imu_plotter.imu_publ:main',
                'angular_acceleration_ya_pi_ro = imu_plotter.angular_acceleration_ya_pi_ro:main',
                'acceleration_pi_ro = imu_plotter.acceleration_pi_ro:main',
                'magnetic_ya = imu_plotter.magnetic_ya:main',
                'kalman_filtter = imu_plotter.kalman_filtter:main',
                'visualizer = imu_plotter.visualizer:main'
        ],
    },
)
