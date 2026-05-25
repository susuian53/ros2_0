#!/usr/bin/env python3
"""
Cyberdog 摄像头 Web 查看器
简单的 HTTP 服务器，提供实时摄像头画面
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import http.server
import socketserver
import threading
import io
import time

class CameraHandler(http.server.BaseHTTPRequestHandler):
    """HTTP 请求处理器"""
    
    def log_message(self, format, *args):
        """禁用默认日志"""
        pass
    
    def do_GET(self):
        """处理 GET 请求"""
        if self.path == '/' or self.path == '/index.html':
            self.send_html()
        elif self.path.startswith('/image.jpg'):
            self.send_image()
        else:
            self.send_error(404)
    
    def send_html(self):
        """发送 HTML 页面"""
        html = '''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Cyberdog 摄像头</title>
    <style>
        body { 
            margin: 0; 
            background: #1a1a1a; 
            display: flex; 
            justify-content: center; 
            align-items: center; 
            min-height: 100vh;
        }
        img { 
            max-width: 100%; 
            max-height: 100vh;
            border: 2px solid #4CAF50;
        }
    </style>
</head>
<body>
    <img id="cam" src="/image.jpg" alt="摄像头画面">
    <script>
        // 每 100ms 刷新图像
        setInterval(() => {
            document.getElementById('cam').src = '/image.jpg?t=' + Date.now();
        }, 100);
    </script>
</body>
</html>'''
        
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', len(html))
        self.end_headers()
        self.wfile.write(html.encode())
    
    def send_image(self):
        """发送图像"""
        global latest_image
        
        if latest_image is None:
            self.send_error(503, 'No image available')
            return
        
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', len(latest_image))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(latest_image)


class CameraWebServer(Node):
    """ROS2 摄像头节点"""
    
    def __init__(self):
        super().__init__('camera_web_server')
        
        self.bridge = CvBridge()
        self.image_count = 0
        
        # 使用 best_effort QoS
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            Image, '/rgb/image_raw', self.image_callback, qos)
        
        self.get_logger().info('摄像头 Web 服务器已启动')
        self.get_logger().info('访问地址: http://localhost:8082')
    
    def image_callback(self, msg):
        """接收摄像头图像"""
        global latest_image
        
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # 编码为 JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            latest_image = buffer.tobytes()
            
            self.image_count += 1
            if self.image_count % 30 == 0:
                self.get_logger().info(f'已接收 {self.image_count} 帧')
                
        except Exception as e:
            self.get_logger().error(f'处理失败: {e}')


# 全局变量存储最新图像
latest_image = None


def run_http_server():
    """运行 HTTP 服务器"""
    PORT = 8082
    with socketserver.TCPServer(("", PORT), CameraHandler) as httpd:
        print(f"HTTP 服务器运行在端口 {PORT}")
        httpd.serve_forever()


def main():
    # 启动 HTTP 服务器线程
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    
    # 启动 ROS2 节点
    rclpy.init()
    node = CameraWebServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
