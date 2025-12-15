import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# グローバル変数としてパブリッシャとサブスクライバ、ブリッジを定義
# クラスを使わないため、グローバル変数としてアクセスできるようにする
r_publisher = None
g_publisher = None
b_publisher = None
image_subscriber = None
cv_bridge = None

def image_callback(msg):
    """
    /image_rawトピックからカラー画像メッセージを受信したときに呼び出されるコールバック関数。
    """
    global r_publisher, g_publisher, b_publisher, cv_bridge

    # 画像メッセージをOpenCVの画像形式（NumPy配列）に変換
    try:
        # カラー画像として変換 (bgr8形式を想定)
        cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        rclpy.logging.get_logger('channel_splitter').error(f'CvBridge変換エラー: {e}')
        return

    # --- チャンネル分解処理 ---
    # cv2.splitを使用して画像をB, G, Rの各チャンネルに分解
    # OpenCVは通常B, G, Rの順
    b_channel, g_channel, r_channel = cv2.split(cv_image)

    # 各チャンネル画像をグレースケールとして処理すると画素値は1チャンネル（0〜255）
    # ROS2のImageメッセージに戻す前に、この1チャンネルのグレースケール画像を
    # ROS2メッセージの形式に変換し直す

    # Rチャンネルの変換とパブリッシュ
    try:
        # グレースケール（mono8）として変換
        r_msg = cv_bridge.cv2_to_imgmsg(r_channel, encoding="mono8")
        r_msg.header = msg.header # タイムスタンプなどのヘッダ情報をコピー
        r_publisher.publish(r_msg)
    except Exception as e:
        rclpy.logging.get_logger('channel_splitter').error(f'Rチャンネルパブリッシュエラー: {e}')

    # Gチャンネルの変換とパブリッシュ
    try:
        g_msg = cv_bridge.cv2_to_imgmsg(g_channel, encoding="mono8")
        g_msg.header = msg.header
        g_publisher.publish(g_msg)
    except Exception as e:
        rclpy.logging.get_logger('channel_splitter').error(f'Gチャンネルパブリッシュエラー: {e}')

    # Bチャンネルの変換とパブリッシュ
    try:
        b_msg = cv_bridge.cv2_to_imgmsg(b_channel, encoding="mono8")
        b_msg.header = msg.header
        b_publisher.publish(b_msg)
    except Exception as e:
        rclpy.logging.get_logger('channel_splitter').error(f'Bチャンネルパブリッシュエラー: {e}')

    # rclpy.logging.get_logger('channel_splitter').info('画像を各チャンネルに分解し、パブリッシュしました。')

def main(args=None):
    """
    メイン関数: ノードの初期化と実行
    """
    global r_publisher, g_publisher, b_publisher, image_subscriber, cv_bridge

    # rclpyの初期化
    rclpy.init(args=args)

    # --- ノードの作成（クラス不使用のため、関数内で作成し、グローバル変数として保持） ---
    # rclpy.create_nodeを使ってノードを作成
    node = rclpy.create_node('channel_splitter_node')

    rclpy.logging.get_logger('channel_splitter').info('チャネル分解ノードを開始します...')

    # --- パブリッシャの作成 ---
    r_publisher = node.create_publisher(Image, 'image/r_channel', 10)
    g_publisher = node.create_publisher(Image, 'image/g_channel', 10)
    b_publisher = node.create_publisher(Image, 'image/b_channel', 10)

    # --- サブスクライバの作成 ---
    # /image_rawトピックをサブスクライブし、image_callback関数を呼び出す
    image_subscriber = node.create_subscription(
        Image,
        '/image_raw',
        image_callback,
        10
    )

    # CvBridgeの初期化
    cv_bridge = CvBridge()

    # Ctrl+Cによる安全なシャットダウンのためのtry-exceptブロック
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('channel_splitter').info('Ctrl+Cが押されました。ノードをシャットダウンします...')
    finally:
        # --- ノード終了処理 ---
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()