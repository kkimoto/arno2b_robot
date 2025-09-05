#--- 最初の準備

# ROS2 humble のインストール
# rosdep のインストール
sudo apt install python3-rosdep2

# ワークスペースを作る
# make next version (b) workspace
mkdir arno2b_ws
cd    arno2b_ws
mkdir src

# rviz2 の準備
# 木元の環境では rviz を他のPCに表示する
# 他のPCで表示を受け付けるようにする
xhost +
# ロボット(ROS2)の PC で遠隔表示をする準備
export DISPLAY=192.168.0.56:0

#--- YVT (3D lidar) と urg3d_node2 package のテスト

# test YVT sensor and urg3d_node2 package
cd arno2b_ws/src
git clone --recursive https://github.com/Hokuyo-aut/urg3d_node2
rosdep update
rosdep install -i --from-paths urg3d_node2
cd ..
# IP アドレスを実機に合わせておく 
# src/urg3d_node2/config/params.yaml
# の次の行
#     ip_address: '192.168.0.10'

# ビルド
colcon build --symlink-install
# YVT-35LX の電源を入れて、ネットワーク接続する。
# test run
source install/setup.bash
ros2 launch urg3d_node2 urg3d_node2.launch.py
# または
bash myscripts/com_001_test1_urgnode2.bash

# rviz2 で確かめる
ros2 topic echo /hokuyo_cloud2|egrep frame
# frame_id: hokuyo3d とでるので
# rviz2 では
# Fixed Frame hokuyo3d
# Add から PointCloud2 /hokuyo_cloud2 を選択する
# またはすでに設定がしているファイルを指定する
cd arno2b_ws
rviz2 -d ~/.rviz2/urg3d_node2.rviz

#--- 統合パッケージを作る
#    new package arno2_robot
cd arno2b_ws/src
ros2 pkg create --build-type ament_python arno_robot
cd arno_robot
mkdir -p config launch resource
cd ../..
echo "arno_robot" > src/arno_robot/resource/arno_robot

#--- urg3d_node2 package を arno_robot package の設定で動かす
#    arno_robot パッケージに設定を入れて動かす

# arno_robot パッケージ設定の編集
# src/arno_robot/setup.cfg
# src/arno_robot/setup.py

cp src/urg3d_node2/launch/urg3d_node2.launch.py src/arno_robot/launch/urg3d_node2_settingA.launch.py
cp src/urg3d_node2/config/params.yaml src/arno_robot/config/params_yvt.yaml

# 'params_yvt.yaml' は src/arno_robot/launch/urg3d_node2_settingA.launch.py の中で指定する。
#下のソースの4行目のようにする
#    config_file_path = os.path.join(
#        get_package_share_directory('arno_robot'),
#        'config',
#        'params_yvt.yaml'   # keep filename; adjust if you rename
#    )

# src/arno_robot/config/params_yvt.yaml のパラメータを変える。例えば
#
#   interlace_h: 4

# arno_robot パッケージのビルド
colcon build --symlink-install --packages-select arno_robot

#または全部作り直す # rm -f は強制的に消すので扱いは気をつけて
rm -rf build install log
colcon build --symlink-install

# arno_robot パッケージを独自 launch ファイルで実行
ros2 launch arno_robot urg3d_node2_settingA.launch.py

# rviz2 でインターレースが入っているところが見えるはず

#--- 2D lidar urg_node2 を動かす
# 手元の UST-20LX で実験しました

# インストール
cd arno2b_ws/src

git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
rosdep update
rosdep install -i --from-paths urg_node2

# ビルド
cd ../../arno2b_ws
colcon build --symlink-install

# IP アドレスを変える 下のファイルを編集
# src/urg_node2/config/params_ether.yaml
# src/urg_node2/config/params_ether_2nd.yaml

# テスト 2台動かす
ros2 launch urg_node2 urg_node2_2lidar.launch.py

# トピックを見る
ros2 topic list
# 2つ見える
# /scan_1st
# /scan_2nd

# rviz2 では
# /scan_1st のフレームは laser
# /scan_2nd のフレームは laser_2nd
# このままでは同時には見えない

#--- urg_node2 を arno_robot の設定と launcher で動かす。

# 次のファイルを作った
# src/arno_robot/launch/urg_node2_2lidar.launch.py
# src/arno_robot/config/params_default.yaml

# params_default.yaml にはすべての既定値を詰め込んでいく。
# この時点では laser1, laser2, yvt_3d のパラメータがある
# src/arno_robot/launch/urg_node2_2lidar.launch.py は params_default.yaml を読み込んでいる。

colcon build --symlink-install
source install/setup.bash
ros2 launch arno_robot urg_node2_2lidar.launch.py

# rviz2 では
# frame laser_frame_1 で /scan_1st topic が見える
# frame laser_frame_2 で /scan_1nd topic が見える



--------------------------以下はまだ


# -- 2台の 2D lidar を1つのフレームに合わせる laser_scan_merger パッケージの導入
# 次の github 参考
https://github.com/BruceChanJianLe/laser_scan_merger

cd arno2b_ws/src
git clone https://github.com/BruceChanJianLe/laser_scan_merger.git

cd ..   (cd arno2b_ws)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# または
bash myscripts/com_build_all.bash

# 設定
# 設定ファイル  src/arno_robot/config/param_laser_scan_merger.yaml
# 次の topic と frame を指定する
#    scan_topics: ["/scan_1st", "/scan_2nd"]
#    frames: ["laser_frame_1", "laser_frame_2"]

# laser_frame_1, laser_frame_2 と base_link の関係を定義する
# static_starnsforms.launch をつくる
# src/arno_robot/launch/static_transforms.launch.py
# ロボットの Arno と違って 83 mm 前と 83 mm 後ろの位置でに前後に向けられている。
