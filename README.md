# chinorobo_ros2_opencv_01
## パッケージ説明
知能ロボットシステムコース, 5R実習科目「ロボット知能化演習」のOpenCV実習パッケージ01 
## 必要な外部パッケージのインストール
```
$sudo apt update
$sudo apt install ros-humble-image-pipeline
$sudo apt install numpy==1.24.4 opencv-python==4.8.1.78 cv-bridge
```

## 実習用リポジトリのクローン
```
$cd ros2_ws/src
$git clone https://github.com/tominagalab/chinorobo_ros2_opencv_01.git
```

## 授業内容
### OpenCVをROS2で使用する
1. 画像データ，画像処理とは
1. OpenCVについて
### リマップ機能を活用したROS2システム構築
#### トピック名のリマップ
ノード実行時にコマンドライン引数でノードが扱うトピック名を置換できる．
パブリッシャー・サブスクライバーのどちらでも置換可能である．
以下に例を示す．
```
$ros2 run chinorobo_ros2_opencv_01 image_grayscale2binary_node --ros-args --remap /image_grayscale:=/image/r_channel
```
リマップしたいトピックを追加する場合，--remapが再度必要になる．以下に例を示す．
```
$ros2 run chinorobo_ros2_opencv_01 image_grayscale2binary_node --ros-args --remap /image_grayscale:=/image/r_channel --remap /image_binary:=/image_bin_r_channel
```

#### ノード名のリマップ
ノード実行時にコマンドライン引数でノード名を置換できる．
以下に例を示す．
```
$ros2 run chinorobo_ros2_opencv_01 image_grayscale2binary_node --ros-args --remap __node:=image_gray2bin_node1
```
#### ノード名もトピック名もリマップする場合
各リマップに対して--remapをつけて実行することになる．
以下に例を示す．
```
$ros2 run chinorobo_ros2_opencv_01 image_grayscale2binary_node --ros-args --remap __node:=image_gray2bin_node1　--remap /image_grayscale:=/image/r_channel --remap /image_binary:=/image_bin_r_channel
```