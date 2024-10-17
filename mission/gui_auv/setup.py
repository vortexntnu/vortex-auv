from setuptools import setup

PACKAGE_NAME = 'auv_gui'

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sondre',
    maintainer_email='sondre95556888@gmail.com',
    description='AUV GUI for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auv-gui = auv_gui.auv_gui:main',
        ],
    },
)
