from setuptools import setup

package_name = 'vora'
submodules = 'vora/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dokyun',
    maintainer_email='dokyun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_vora = vora.run_vora:main',
            'neato_control = vora.neato_control:main',
            'test = vora.test:main',
            'voice_handler = vora.voice_handler:main',
            'test_apriltag = vora.test_apriltag:main',
        ],
    },
)
