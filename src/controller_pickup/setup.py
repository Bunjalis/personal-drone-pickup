from setuptools import find_packages, setup

package_name = 'controller_pickup'

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
    maintainer='mitchell',
    maintainer_email='mitch.torok@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'main = controller_pickup.main:main',
            'delay_estimator_node = controller_pickup.delay_estimator_node:main'
        ],
    },
)
