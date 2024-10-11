from setuptools import setup

package_name = 'guidance_test'

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
    maintainer='badawi',
    maintainer_email='badawi@todo.todo',
    description='A test package for guidance functionality',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance_test = guidance_test.guidance_test:main',
        ],
    },
)
