from setuptools import find_packages, setup


package_name = 'keyence_profile_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/keyence_profile_replay.launch.py']),
        ('share/' + package_name + '/test/fixtures', ['test/fixtures/nominal_profiles.jsonl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyence_profile_replay = keyence_profile_ros2.profile_replay:main',
            'ljx8_profile_driver = keyence_profile_ros2.ljx8_profile_driver:main',
        ],
    },
)
