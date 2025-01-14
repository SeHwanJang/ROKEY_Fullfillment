from setuptools import find_packages, setup

package_name = 'control_tower'

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
    maintainer='rokey',
    maintainer_email='ssm06081@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_tower=project.control_tower:main',
            'server=project.control_tower_server:main',
            'camera=project.camera:main',
            'main_control=project.subscription:main',
            'fix_rotation=project.fix_rotation:main',
        ],
    },
)
