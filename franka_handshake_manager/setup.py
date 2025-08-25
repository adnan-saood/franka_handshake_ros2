from setuptools import find_packages, setup

package_name = 'franka_handshake_manager'

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
    maintainer='adnan',
    maintainer_email='adnan.saood@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'franka_handshake_manager = franka_handshake_manager.main:main'
        ],
    },
)
