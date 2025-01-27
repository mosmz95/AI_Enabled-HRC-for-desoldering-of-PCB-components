from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'safety_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob('configs/*.task')),
        (os.path.join('share', package_name, 'configs'), glob('configs/*')),
    ],
    install_requires=['setuptools','pyrealsense2','opencv-python','opencv-python-headless'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='mostafazarei1995mz@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            "handlandmark =safety_check.scripts.hand_landmarker_publisher:main",
            "gui = safety_check.scripts.graphical_user_interfacev1:main",
            "component_selection_client = safety_check.scripts.component_selection_gui_client:main",
            "component_detection_server = safety_check.scripts.component_detection_server:main"

        ],
    },
)
