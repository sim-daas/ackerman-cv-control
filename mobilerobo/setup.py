import glob
import os
from setuptools import find_packages, setup

package_name = 'mobilerobo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*') if os.path.isdir('launch') else []),
        ('share/' + package_name + '/config', glob.glob('config/*') if os.path.isdir('config') else []),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*') if os.path.isdir('worlds') else []),
        ('share/' + package_name + '/maps', glob.glob('maps/*') if os.path.isdir('maps') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='aitechmanml@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_navigate    = mobilerobo.auto_navigate:main',
            'perception_node  = mobilerobo.perception_node:main',
            'control_node     = mobilerobo.control_node:main',
        ],
    },
)
