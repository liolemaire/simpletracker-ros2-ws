import os
from glob import glob
from setuptools import setup

package_name = 'simple_tracker_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    include_package_data=True,
    data_files=[    ('share/ament_index/resource_index/packages',        ['resource/' + package_name]),
                    ('share/' + package_name, ['package.xml']),
                    (os.path.join('share', package_name, 'chatgpt.model/'), glob('chatgpt.model/*.*')),
                    (os.path.join('share', package_name, 'chatgpt.model/assets/'), glob('chatgpt.model/assets/*.*')),
                    (os.path.join('share', package_name, 'chatgpt.model/variables/'), glob('chatgpt.model/variables/*.*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='liolemaire@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_service = simple_tracker_kinematics.kinematics_service:main',
            'kinematics_client = simple_tracker_kinematics.kinematics_client:main',
        ],
    },
)
