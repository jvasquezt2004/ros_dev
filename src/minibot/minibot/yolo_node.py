#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publicador (opcional)
        self.publisher_ = self.create_publisher(Image, '/minibot/yolo_result', 10)
        
        self.bridge = CvBridge()
        # Cargar modelo (se descargará solo la primera vez si no existe)
        self.model = YOLO("yolov8n.pt") 
        self.get_logger().info("Nodo YOLO iniciado con Ultralytics (modo nativo)")
        self.frame_count = 0
        self.process_rate = 5

    def image_callback(self, msg):
        try:
            # 1. ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.circle(cv_image, (30, 30), 10, (255, 0, 0), -1)

            # 2. Inferencia (verbose=False para no llenar la terminal de texto)
            results = self.model(cv_image, verbose=False, conf=0.1)

            # 3. Dibujar resultados
            annotated_frame = results[0].plot()

            # 4. Mostrar en pantalla
            cv2.imshow("Minibot Vision", annotated_frame)
            cv2.waitKey(1)

            # 5. Publicar
            processed_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.publisher_.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()