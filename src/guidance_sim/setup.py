import os
from glob import glob

from setuptools import setup

package_name = 'guidance_sim'


def model_data_files():
    """Install models/ keeping its directory structure."""
    out = []
    for root, _dirs, files in os.walk('models'):
        if not files:
            continue
        dest = os.path.join('share', package_name, root)
        out.append((dest, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.parm')),
        ('share/' + package_name + '/tools', glob('tools/*.py')),
    ] + model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthik',
    maintainer_email='aeroboys69@gmail.com',
    description='Two-quadcopter guidance testbed for AS5570.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'takeoff = guidance_sim.takeoff:main',
            'land = guidance_sim.takeoff:main_land',
            'takeoff_both = guidance_sim.takeoff:main_both',
            'land_both = guidance_sim.takeoff:main_land_both',
            'engage = guidance_sim.engage:main',
            'offline = guidance_sim.offline:main',
            'plot_run = guidance_sim.plot_run:main',
            'sweep = guidance_sim.offline:main_sweep',
        ],
    },
)
