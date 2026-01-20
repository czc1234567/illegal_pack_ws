from setuptools import find_packages, setup

package_name = 'fishbot_navigator'

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
    maintainer='czc',
    maintainer_email='129060653+czc1234567@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'single_goal_nav = fishbot_navigator.single_goal_nav:main',
            'multi_goal_nav = fishbot_navigator.multi_goal_nav:main',
            'patrol_and_detect = fishbot_navigator.patrol_and_detect:main',
        ],
    },
)
