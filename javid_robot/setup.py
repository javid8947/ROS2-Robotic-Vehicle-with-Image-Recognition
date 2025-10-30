from setuptools import find_packages, setup

package_name = 'javid_robot'

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
    maintainer='javid',
    maintainer_email='javid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera = javid_robot.camera:main",
            "ai = javid_robot.ai:main",
            "arduino = javid_robot.arduino:main",
        ],
    },
)
