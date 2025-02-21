from setuptools import find_packages, setup

package_name = 'turtle_hunter'

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
    maintainer='lucifer',
    maintainer_email='rajasekarand375@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "deity = turtle_hunter.deity:main",
            "hunter = turtle_hunter.hunter:main"
        ],
    },
)
