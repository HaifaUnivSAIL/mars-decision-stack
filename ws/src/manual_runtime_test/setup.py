from setuptools import find_packages, setup

package_name = 'manual_runtime_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_runtime_test.launch.py']),
        ('share/' + package_name + '/config', ['config/keyboard.yaml', 'config/scripted_smoke_test.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guy Sassy',
    maintainer_email='guy-sassy@example.com',
    description='Interactive manual runtime test package for the Mars Decision Stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = manual_runtime_test.keyboard_teleop_node:main',
        ],
    },
)
