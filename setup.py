from setuptools import setup
from glob import glob
import os

package_name = 'stereo_camera_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shye',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_ip_publisher = stereo_camera_pipeline.publish_stereo:main',
            'stereo_rectifier_node = stereo_camera_pipeline.rectify_and_publish:main',
            'stereo_processor_node = stereo_camera_pipeline.stereo_processor_node:main',
        ],
    },
)
