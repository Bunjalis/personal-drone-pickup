from setuptools import setup

package_name = 'motion_capture_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_capture_publisher_node = motion_capture_publisher.motion_capture_publisher_node:main'
        ],
    },
)
