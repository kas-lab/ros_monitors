from setuptools import find_packages, setup


package_name = 'ros_monitors_functions'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rezenders',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Common functions to process monitoring information',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
)
