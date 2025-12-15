import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# クラスを使わない基本的な構造
# グローバル変数としてパブリッシャとブリッジを定義
image_publisher = None
cv_bridge = None

# 【設定】二値化処理で使用する閾値（しきい値）
# 適切な値は画像によって異なりますが、ここでは127を使用します。
BINARY_THRESHOLD = 127

def image_callback(msg: Image):
    """
    /grayscale_imageトピックからグレースケール画像メッセージを受信したときに呼び出されるコールバック関数。
    """
    global image_publisher, cv_bridge

    # 1. 画像メッセージをOpenCV形式（NumPy配列）に変換
    try:
        # 入力はグレースケール画像（mono8形式）を想定
        # 二値化処理はグレースケール画像（1チャンネル）に対して行います。
        input_cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
    except Exception as e:
        rclpy.logging.get_logger('grayscale_to_binary').error(f'CvBridge変換エラー（受信側）: {e}')
        return

    # ----------------------------------------------------
    # *** 二値化処理の実装 ***
    # ----------------------------------------------------
    
    # cv2.threshold を使用して、画像を二値化する
    # ret: 実際に使用された閾値 (ここでは BINARY_THRESHOLD と同じ)
    # processed_cv_image: 二値化された画像（0または255の値を持つ）
    ret, binary_image = cv2.threshold(
        input_cv_image,             # 処理対象のグレースケール画像
        BINARY_THRESHOLD,           # 閾値
        255,                        # 最大値
        cv2.THRESH_BINARY           # 二値化の種類
    )
    
    # 処理後のエンコーディングを指定
    # 二値化画像は1チャンネルであり、'mono8' エンコーディングを使用します。
    output_encoding = 'mono8'
    
    # ----------------------------------------------------

    # 2. 処理済み（二値化済み）画像をROSのImageメッセージに変換
    try:
        output_msg = cv_bridge.cv2_to_imgmsg(binary_image, encoding=output_encoding)
        output_msg.header = msg.header # タイムスタンプなどのヘッダ情報をコピー
        
        # 3. 処理済み画像をパブリッシュ
        image_publisher.publish(output_msg)
        
        # rclpy.logging.get_logger('grayscale_to_binary').info('画像を二値化し、パブリッシュしました。')

    except Exception as e:
        rclpy.logging.get_logger('grayscale_to_binary').error(f'CvBridge変換エラー（パブリッシュ側）: {e}')


def main(args=None):
    """
    メイン関数: ノードの初期化と実行
    """
    global image_publisher, cv_bridge
    
    rclpy.init(args=args)
    
    # ノードの作成
    node = rclpy.create_node('grayscale_to_binary_node')
    rclpy.logging.get_logger('grayscale_to_binary').info(f'グレースケールto二値化ノードを開始します (閾値: {BINARY_THRESHOLD}) ...')

    # --- パブリッシャの作成 ---
    # 二値化後の画像を出力するトピック
    image_publisher = node.create_publisher(Image, 'image_binary', 10)

    # --- サブスクライバの作成 ---
    # グレースケール画像を入力とするトピックをサブスクライブ
    image_subscriber = node.create_subscription(
        Image,
        '/grayscale_image', # サブスクライブするトピック名
        image_callback,
        10
    )

    # CvBridgeの初期化
    cv_bridge = CvBridge()

    # Ctrl+Cに対する安全なシャットダウン処理
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grayscale_to_binary').info('Ctrl+Cが押されました。ノードをシャットダウンします...')
    finally:
        # ノード終了処理
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()