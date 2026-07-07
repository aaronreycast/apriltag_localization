from setuptools import find_packages, setup

package_name = 'apriltag_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/apriltag_localization']),
        ('share/apriltag_localization', ['package.xml']),
        ('share/apriltag_localization/config', ['config/tag_map.yaml']),
        ('share/apriltag_localization/launch', ['launch/localize.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaronreycast',
    maintainer_email='aaronreycast@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vessel_localizer = apriltag_localization.localizer_node:main',
        ],
    },
)
