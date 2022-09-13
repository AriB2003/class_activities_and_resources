from setuptools import setup

package_name = 'in_class_day04_solutions'

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
    maintainer='pruvolo',
    maintainer_email='paullundyruvolo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_approach_starter = in_class_day04_solutions.wall_approach_starter:main',
            'wall_approach = in_class_day04_solutions.wall_approach:main',
            'wall_approach_fancy = in_class_day04_solutions.wall_approach_fancy:main'
        ],
    },
)
