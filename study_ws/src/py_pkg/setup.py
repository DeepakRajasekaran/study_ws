from setuptools import find_packages, setup

package_name = 'py_pkg'

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
    maintainer_email='lucifer@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = py_pkg.first_node:main",
            "news_station = py_pkg.news_station:main",
            "radio = py_pkg.radio:main",
            "sum_server = py_pkg.sum_server:main",
            "addTwoInt_noop = py_pkg.client_sumServer_noop:main",
            "client_sumServer = py_pkg.client_sumServer:main",
            "HardwareStatusPublisher = py_pkg.hardwareTest:main",
            "client_node = py_pkg.client_RectangleArea:main",
            "RectangleArea_server = py_pkg.RectangleArea_server:main",
            "numberPublisher = py_pkg.numberPublisher",
            "numberCounter = py_pkg.numberCounter",
        ],
    },
)
