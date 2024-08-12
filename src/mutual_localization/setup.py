from setuptools import find_packages, setup

package_name = 'mutual_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/point_reg.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dssl',
    maintainer_email='weibingchuan20@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mutual_localization = mutual_localization.mutual_localization:main'
        ],
    },
)
