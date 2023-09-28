from setuptools import find_packages, setup

# Packageの名前
package_name = 'whill_navi2'

# このPackageをビルドするメソッド
# 引数1：Packageの名前
# 引数2：依存するPackageを探す。しかし「test」に関するPackageを除外する。
# 引数3：インストールする際に、データファイルの保存先。例：config/param, launchなど
# 引数4：インストールする際に、依存するライブラリー
# 引数5：zipアーカイブ形式を有効にするかどうか。アーカイブとは、URL：https://wa3.i-3-i.info/word1247.html
# 引数6~9：開発者情報やライセンス情報
# 引数10：テスト方式
# 引数11：インストールするスクリプトの居場所。例：実行ファイルの名前 = パッケージ名.スクリプト名(拡張子なし).main(メソッド名)

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
