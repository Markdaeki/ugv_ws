from setuptools import setup
import os
from glob import glob

package_name = 'rosmaster_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 기본
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # SLAM 파라미터 파일 설치
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup for Rosmaster UGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'scan_front_filter_node = rosmaster_bringup.scan_front_filter_node:main',
        ],
    },
)
