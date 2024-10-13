from setuptools import setup, find_packages

package_name = 'guidance_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='badawi',
    maintainer_email='badawi@todo.todo',
    description='Package for guidance node implementing PID control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance = guidance_pkg.guidance:main',  # Ensure this is correctly pointing to the main function
        ],
    },
)
