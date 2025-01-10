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
            'view = control_tower.control_tower:main',
            'view_server = control_tower.control_tower_server:main',
            'cam = control_tower.camera:main',
            'conbelt_server = control_tower.conbelt_server:main',
            
            'move = control_tower.move:main',
            
            
            'temp_server = control_tower.server:main',
            'temp_client = control_tower.client:main',
            
            
            'fix = control_tower.fix_rotation:main',
            'fix_sub = control_tower.fix_rotation_sub:main',
        ],
    },
)
