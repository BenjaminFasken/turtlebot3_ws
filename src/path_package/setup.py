from setuptools import find_packages, setup

package_name = 'path_package'

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
    maintainer='benjamin',
    maintainer_email='benjaminfasken@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_node = path_package.path_node:main',
            'map_filter_node = path_package.map_filter_node:main'
        ],
    },
)
