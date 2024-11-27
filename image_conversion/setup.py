from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_conversion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeni',
    maintainer_email='seeni@example.com',
    description='A package for converting images from color to greyscale and vice versa.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_conversion_node = image_conversion.image_conversion_node:main',
        ],
    },
)


