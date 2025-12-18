import os
from glob import glob
from setuptools import setup

package_name = 'kalmanfilter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryosuke',
    maintainer_email='ryosuke@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # 2. スクリプトをノードとして実行可能にするための設定
    entry_points={
        'console_scripts': [
            # '実行可能ファイル名 = パッケージ名.ディレクトリ名.ファイル名:main'
            'clock_publisher = ' + package_name + '.scripts.clock_publisher:main',
        ],
    },
)