from setuptools import find_packages, setup
import os
from glob import glob
from ament_index_python.packages import get_package_share_directory

# Packageの名前
package_name = 'whill_navi2'
SHARE_DIR = os.path.join('share', package_name)

# このPackageをビルドするメソッド
# 引数1：Packageの名前
# 引数2：バージョン
# 引数3：依存するPackageを探す。しかし「test」に関するPackageを除外する。
# 引数4：インストールファイルのアドレスとインストール先を指定する。Tupleの1つ目の要素はインストール先を指定し、2つ目の要素の型はリストであり、インストールファイルのアドレスを指定する。
# 引数5：インストールする際に、依存するライブラリー
# 引数6：zipアーカイブ形式を有効にするかどうか。アーカイブとは、URL：https://wa3.i-3-i.info/word1247.html
# 引数7~10：開発者情報やライセンス情報
# 引数11：テスト方式
# 引数12：インストールするスクリプトの居場所。例：実行ファイルの名前 = パッケージ名.スクリプト名(拡張子なし).main(メソッド名)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, 
            ['package.xml']),
        (os.path.join(SHARE_DIR, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join(SHARE_DIR, 'launch', 'include'), 
            glob(os.path.join('launch', 'include', '*launch.py'))),
        (os.path.join(SHARE_DIR, 'config', 'params'), 
            glob(os.path.join('config', 'params', '*.yaml'))),
        (os.path.join(SHARE_DIR, 'config', 'rviz2'), 
            glob(os.path.join('config', 'rviz2', '*.rviz'))),
        (os.path.join(SHARE_DIR, 'config', 'launch_arg'), 
            glob(os.path.join('config', 'launch_arg', '*.yaml'))),
        (os.path.join(SHARE_DIR, 'config', 'urdf'), 
            glob(os.path.join('config', 'urdf', '*.urdf'))),       
        (os.path.join(SHARE_DIR, 'config', 'xacro'), 
            glob(os.path.join('config', 'xacro', '*.xacro')))        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='han',
    maintainer_email='k895657@kansai-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_dir_node = whill_navi2.make_dir_node:main'
        ],
    },
)

try:
    os.link(
        os.path.join(
            get_package_share_directory("whill_navi2"),
            "config", "params", "make_dir_node_params.yaml"
        ),
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)),
            "data_dir_manager"
        )
    )
except OSError as e:
    pass
