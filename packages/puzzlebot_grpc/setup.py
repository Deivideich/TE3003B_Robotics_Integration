from setuptools import setup

package_name = 'puzzlebot_grpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='gRPC interface for Puzzlebot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gRPC_handler = puzzlebot_grpc.gRPC_handler:main'
        ],
    },
)
