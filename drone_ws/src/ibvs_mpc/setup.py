from setuptools import find_packages, setup

package_name = 'ibvs_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aryan Singh',
    maintainer_email='aryan7109singh@gmail.com',
    description='Image-Based Visual Servoing and MPC package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = ibvs_mpc.aruco_detector:main',
            'ibvs_controller = ibvs_mpc.ibvs_controller:main',
            'gz_tf_broadcaster = ibvs_mpc.gz_tf_broadcaster:main',
        ],
    },
)