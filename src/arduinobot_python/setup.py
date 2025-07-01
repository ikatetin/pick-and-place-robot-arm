from setuptools import find_packages, setup

package_name = 'arduinobot_python'

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
    maintainer='yoga-slim-pro-2',
    maintainer_email='yoga-slim-pro-2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = arduinobot_python.camera_node:main',
            'yolo_node = arduinobot_python.yolo_node:main'
        ],
    },
)
