# chinorobo_ros2_opencv_01
## パッケージ概要（Package description）
知能ロボットシステムコース5R実習科目「ロボット知能化演習」の画像処理実習パッケージその1
## インストール方法（How to install & setup）
### 必要パッケージのインストール（install requires）
```
$sudo apt update
$sudo apt install ros-humble-image-pipeline
$sudo apt install numpy==1.24.4 opencv-python==4.8.1.78 cv-bridge
```
### 本パッケージのダウンロード方法（Download the repository）
```
$cd ros2_ws/src
$git clone https://github.com/tominagalab/chinorobo_ros2_opencv_01.git
```

## ノード仕様（Nodes）
### __*image_splitter_node*__
3チャンネル画像をチャンネル分解して1チャンネル画像×3を出力するノード
- サブスクライバー（Subscribers）
  - *image_raw* : 3チャンネルの入力画像．
- パブリッシャー（Publishers）
  - *image/b_channnel* : 青チャンネル強度画像（1チャンネルグレースケール画像）．
  - *image/g_channnel* : 緑チャンネル強度画像（1チャンネルグレースケール画像）．
  - *image/r_channnel* : 赤チャンネル強度画像（1チャンネルグレースケール画像）．
- ノード起動方法（how to excute）  
```
ros2 run chinorobo_ros2_opencv_01 image_splitter_node
```  
### __*image_grayscale2binary_node*__
1チャンネル画像を閾値に従って二値化処理し，１チャンネル二値化画像を出力する．
- サブスクライバー（Subscribers）
  - *image_grayscale*: 1チャンネルグレースケール画像
- パブリッシャー（Publishers）
  - *image_binary*: 1チャンネル二値化画像
- ノード起動方法（how to excute）  
```
ros2 run chinorobo_ros2_opencv_01 image_grayscale2binary_node
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