from setuptools import setup

package_name = 'py_udemy_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adrian Raiser',
    maintainer_email='adrian@raiser.dev',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = py_udemy_pkg.my_first_node:main",
            "robot_news_station = py_udemy_pkg.robot_news_station:main",
            "smartphone = py_udemy_pkg.smartphone:main",
            "number_publisher = py_udemy_pkg.number_publisher:main",
            "number_counter = py_udemy_pkg.number_counter:main"
        ],
    },
)
