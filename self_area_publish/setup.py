from setuptools import find_packages, setup

package_name = 'self_area_publish'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryosuke',
    maintainer_email='ryosuke@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
            'console_scripts': [
                # [実行コマンド名] = [Pythonモジュール名].[ファイル名から拡張子を除いたもの]:[main関数名]
                'self_area_pub = self_area_publish.run:main'
        ],
    },
)
