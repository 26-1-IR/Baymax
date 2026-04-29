import os
from glob import glob
from setuptools import find_packages, setup


def get_files_recursively(directory):
    """Recursively get all files in a directory."""
    files = []
    if os.path.isdir(directory):
        for root, dirs, filenames in os.walk(directory):
            for filename in filenames:
                files.append(os.path.join(root, filename))
    return files


package_name = 'autonomous_parking'

# Collect all files from directories
model_files = get_files_recursively('models')
media_files = get_files_recursively('media')

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# launch 파일
launch_files = glob('launch/*.launch.py')
if launch_files:
    data_files.append((os.path.join('share', package_name, 'launch'), launch_files))

# config 파일
config_files = glob('config/*.yaml')
if config_files:
    data_files.append((os.path.join('share', package_name, 'config'), config_files))

# world 파일
world_files = glob('worlds/*.world')
if world_files:
    data_files.append((os.path.join('share', package_name, 'worlds'), world_files))

# urdf 파일
urdf_files = glob('urdf/*.urdf')
if urdf_files:
    data_files.append((os.path.join('share', package_name, 'urdf'), urdf_files))

# model 파일들
if model_files:
    # Group files by their directory structure
    model_dirs = {}
    for fpath in model_files:
        dpath = os.path.dirname(fpath)
        if dpath not in model_dirs:
            model_dirs[dpath] = []
        model_dirs[dpath].append(fpath)

    for dpath, flist in model_dirs.items():
        target_dir = os.path.join('share', package_name, dpath)
        data_files.append((target_dir, flist))

# media 파일들
if media_files:
    media_dirs = {}
    for fpath in media_files:
        dpath = os.path.dirname(fpath)
        if dpath not in media_dirs:
            media_dirs[dpath] = []
        media_dirs[dpath].append(fpath)

    for dpath, flist in media_dirs.items():
        target_dir = os.path.join('share', package_name, dpath)
        data_files.append((target_dir, flist))

# map 파일
map_files = glob('maps/*.pgm') + glob('maps/*.yaml')
if map_files:
    data_files.append((os.path.join('share', package_name, 'maps'), map_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    entry_points={
        'console_scripts': [
            'parking_node = autonomous_parking.parking_node:main',
            'vision_node = autonomous_parking.vision_node:main',
            'decision_node = autonomous_parking.decision_node:main',
        ],
    },
)
