from setuptools import find_packages, setup

package_name = 'ppg_planner'

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
    maintainer='leev',
    maintainer_email='yyootttaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'ppg_planner = {package_name}.point_in_poly:main',
            f'ppg_planner_quad = {package_name}.point_in_poly_quad_tree:main',
            f'test_client = {package_name}.test_client:main'
        ],
    },
)
