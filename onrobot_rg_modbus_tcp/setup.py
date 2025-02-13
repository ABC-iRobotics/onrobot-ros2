from setuptools import find_packages, setup

package_name = 'onrobot_rg_modbus_tcp'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Makány András',
    maintainer_email='andras.makany@irob.uni-obuda.hu',
    description='A stack to communicate with OnRobot RG grippers using the Modbus/TCP protocol. Based on Takuya Kiyokawa\'s package.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
