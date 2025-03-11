from setuptools import setup

package_name = 'bidirectional_search'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Babanpreet Singh',
    maintainer_email='bsa74@sfu.ca',
    description='A ROS2 package for bidirectional search algorithm in Python done by CMPT415 students in Spring 2025',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bidirectional_search = bidirectional_search.search_node:main'
        ],
    },
)
