from setuptools import setup

package_name = 'milestone5'

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
    maintainer='Andrew Scouten',
    maintainer_email='yzb2@txstate.edu',
    description='Whisper, Espeak, and LLaMA on an NVIDIA Jetson Xavier NX with a publisher / subscriber to the TurtleBot3.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
