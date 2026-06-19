from setuptools import setup
package_name = 'state_machine'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samluiz2025',
    maintainer_email='udobasamuel5@gmail.com',
    description='state_machine package',
    license='MIT',
    entry_points={'console_scripts': []},
)
