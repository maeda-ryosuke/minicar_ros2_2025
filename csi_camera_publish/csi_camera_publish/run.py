import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from jetcam.csi_camera import CSICamera 
# その他の必要なライブラリも適宜インポート

class CsiCameraPublisher(Node):
    def __init__(self, width=224, height=224, fps=30):
        super().__init__('csi_camera_publisher')

        # パブリッシャーの設定
        # トピック名: /camera/image_raw
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # カメラの初期化（元のコードから流用）
        self.camera = CSICamera(capture_device=1, width=width, height=height, capture_fps=fps)
        self.get_logger().info(f'CSICamera initialized: {width}x{height} @ {fps}fps')

        # 読み取り周期の設定
        # FPSに応じてタイマー周期を計算
        timer_period = 1.0 / fps  # 例: 30FPSなら 1/30秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        タイマーで定期的に実行されるコールバック関数。
        カメラから画像を読み込み、ROS 2メッセージに変換してパブリッシュする。
        """
        try:
            # 1. カメラから画像を読み込む
            image = self.camera.read() # numpy.ndarray (BGR)
            
            # 画像のチェック (デバッグ用)
            if image is None:
                self.get_logger().warn('Failed to read image from camera.')
                return

            # 2. OpenCVの画像 (numpy array) を ROS 2の Image メッセージに変換
            # エンコーディング: 'bgr8' (cv2.imread() のデフォルト)
            ros_image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            
            # タイムスタンプを設定 (重要)
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 3. パブリッシュ
            self.image_pub.publish(ros_image_msg)
            # self.get_logger().info('Published image frame.')

        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')

    def destroy_node(self):
        """
        ノード終了時のクリーンアップ処理
        """
        self.get_logger().info('Stopping camera and destroying node...')
        try:
            # カメラの安全な停止（元のコードから流用）
            self.camera.running = False
            time.sleep(0.5)
            self.camera.cap.release()
            self.get_logger().info("Camera released.")
        except Exception as e:
            self.get_logger().warn(f"Error during camera release: {e}")
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 元のコードの FPS 設定に基づき、カメラパラメータを設定
    # 適切な幅/高さ/FPSを設定してください
    WIDTH = 224
    HEIGHT = 224
    # 元コードの fps_type のロジックをここに移植し、適切なFPSを決定する
    FPS = 30 # Orin/AGX Orin の場合
    # FPS = 30 # その他 の場合
    
    publisher_node = CsiCameraPublisher(width=WIDTH, height=HEIGHT, fps=FPS)
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()