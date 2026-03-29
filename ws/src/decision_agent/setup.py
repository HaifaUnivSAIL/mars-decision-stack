from setuptools import find_packages, setup

package_name = 'decision_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/decision_agent.launch.py']),
        ('share/' + package_name + '/config', ['config/policy.yaml', 'config/mission.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guy Sassy',
    maintainer_email='guy-sassy@example.com',
    description='Decision-making package for the Mars Decision Stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_node = decision_agent.policy_node:main',
            'scenario_node = decision_agent.scenario_node:main',
        ],
    },
)
