from setuptools import setup

package_name = 'guidance_action_server'

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
    maintainer='badawi',
    maintainer_email='abubakb@stud.ntnu.no',
    description='Guidance Action Server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guidance_action_server = guidance_action_server.guidance_action_server_node:main',
        ],
    },
)
