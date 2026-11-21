from setuptools import setup


package_name = 'dvrk_endo_calib'

setup(
    name=package_name,
    version='0.1.0',
    description='Collect dVRK ECM waypoints and stereo endoscope images.',
    maintainer='erie_lab',
    maintainer_email='sxy841@case.edu',
    license='Unspecified',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/matlab', ['matlab/open_stereo_calibrator.m', 'matlab/save2yaml.m']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'workspace_collect = dvrk_endo_calib.workspace_collect:main',
            'image_collect = dvrk_endo_calib.image_collect:main',
        ],
    },
)
