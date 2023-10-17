from setuptools import setup

package_name = 'mobipy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florian Hye',
    maintainer_email='florian@hye.dev',
    description='TODO: Package description',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam = mobipy.cam_publisher:main',
        ],
    },
)
