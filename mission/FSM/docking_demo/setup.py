from setuptools import find_packages, setup

PACKAGE_NAME = 'docking_demo'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vetlets',
    maintainer_email='vetlets@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'docking_demo_node = docking_demo.docking_demo_node:main',
            'docking_demo = docking_demo.docking_demo:main',
        ],
    },
)
