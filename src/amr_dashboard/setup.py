from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'amr_dashboard'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Templates
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        # Static CSS
        (os.path.join('share', package_name, 'static', 'css'), glob('static/css/*.css')),
        # Static JS
        (os.path.join('share', package_name, 'static', 'js'), glob('static/js/*.js')),
        # Static assets
        (os.path.join('share', package_name, 'static', 'assets'), glob('static/assets/*')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='Faid',
    maintainer_email='faid@amr.local',
    description='Dashboard Web App untuk kontrol AMR Delivery Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_manager = amr_dashboard.waypoint_manager:main',
            'dashboard_server = amr_dashboard.app:main',
        ],
    },
)
