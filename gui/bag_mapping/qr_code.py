"""
QR Code Scanning Module
Chứa QRScanner class và QRImageSubscriberNode cho việc quét QR code từ ảnh equirectangular
"""
import sys
import os
import io
import contextlib
import warnings
import numpy as np
from concurrent.futures import ThreadPoolExecutor

# Delay import cv2 để tránh segfault
cv2 = None
try:
    import cv2
    _ = cv2.__version__
except (ImportError, SystemError, OSError, AttributeError) as e:
    print(f"Warning: cv2 not available: {e}. QR code scanning will be disabled.")
    cv2 = None
except Exception as e:
    print(f"Warning: cv2 import failed with unexpected error: {e}. QR code scanning will be disabled.")
    cv2 = None

# Try to import pyzbar for QR code scanning
PYZBAR_AVAILABLE = False
QRCODE_SYMBOLS = None
pyzbar = None
try:
    from pyzbar import pyzbar
    from pyzbar.pyzbar import ZBarSymbol
    PYZBAR_AVAILABLE = True
    QRCODE_SYMBOLS = [ZBarSymbol.QRCODE]
except (ImportError, SystemError, OSError, AttributeError) as e:
    PYZBAR_AVAILABLE = False
    QRCODE_SYMBOLS = None
    pyzbar = None
    print(f"Warning: pyzbar not available: {e}. QR code scanning will be disabled.")

# Import geometry utils
GEOMETRY_UTILS_AVAILABLE = False
get_camera_matrix = None
get_extrinsic_matrix = None
camera_to_world = None
cartesian_to_spherical = None
spherical2equirect = None
try:
    from geometry_utils import (
        get_camera_matrix, get_extrinsic_matrix, 
        camera_to_world, cartesian_to_spherical, spherical2equirect
    )
    GEOMETRY_UTILS_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError) as e:
    GEOMETRY_UTILS_AVAILABLE = False
    print(f"Warning: geometry_utils not available: {e}. QR code scanning will be disabled.")

# Try to import ROS2
ROS2_AVAILABLE = False
Node = None
Image = None
Odometry = None
CvBridge = None
try:
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError) as e:
    ROS2_AVAILABLE = False
    print(f"Warning: ROS2 not available: {e}. QR code scanning from bag will be disabled.")


def _decode_qr_silently(gray_image):
    """Wrapper function để decode QRCODE với suppress warnings hoàn toàn"""
    if not PYZBAR_AVAILABLE or pyzbar is None:
        return []
    
    decoded_objects = []
    
    # Suppress warnings ở nhiều mức
    if sys.platform == 'linux':
        # Linux: redirect stderr ở OS level
        saved_stderr = None
        devnull_fd = None
        try:
            devnull_fd = os.open(os.devnull, os.O_WRONLY)
            stderr_fd = sys.stderr.fileno()
            saved_stderr = os.dup(stderr_fd)
            os.dup2(devnull_fd, stderr_fd)
            os.close(devnull_fd)
            devnull_fd = None
            
            # Decode chỉ QRCODE
            try:
                decoded_objects = pyzbar.decode(gray_image, symbols=QRCODE_SYMBOLS)
            except (TypeError, AttributeError):
                # Fallback: decode tất cả rồi filter
                decoded_objects = pyzbar.decode(gray_image)
                decoded_objects = [obj for obj in decoded_objects if obj.type == 'QRCODE']
        except Exception:
            pass
        finally:
            # Restore stderr
            if saved_stderr is not None:
                try:
                    stderr_fd = sys.stderr.fileno()
                    os.dup2(saved_stderr, stderr_fd)
                    os.close(saved_stderr)
                except:
                    pass
            if devnull_fd is not None:
                try:
                    os.close(devnull_fd)
                except:
                    pass
    else:
        # Windows/Mac: dùng redirect_stderr
        stderr_buffer = io.StringIO()
        with contextlib.redirect_stderr(stderr_buffer):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                try:
                    decoded_objects = pyzbar.decode(gray_image, symbols=QRCODE_SYMBOLS)
                except (TypeError, AttributeError):
                    decoded_objects = pyzbar.decode(gray_image)
                    decoded_objects = [obj for obj in decoded_objects if obj.type == 'QRCODE']
                except Exception:
                    decoded_objects = []
    
    return decoded_objects


class QRScanner:
    """Class để quét QR code từ ảnh equirectangular"""
    
    def __init__(self, perspec_size=600, max_workers=4):
        self.perspec_size = perspec_size
        self.max_workers = max_workers
        self._remap_cache = {}
        
        # Kiểm tra tất cả dependencies trước khi enable
        if not PYZBAR_AVAILABLE or not GEOMETRY_UTILS_AVAILABLE or cv2 is None:
            self.enabled = False
        else:
            self.enabled = True
    
    def Equirec2Perspec(self, img: np.ndarray,
                     FOV: float,
                     THETA: float,
                     PHI: float,
                     height: int,
                     width: int) -> np.ndarray:
        """
        Convert equirectangular image to perspective view with caching for speed.
        """
        if cv2 is None or not GEOMETRY_UTILS_AVAILABLE:
            raise RuntimeError("cv2 or geometry_utils not available")
        
        key = (THETA, PHI, FOV, height, width)
        if key in self._remap_cache:
            XY = self._remap_cache[key]
        else:
            # Convert angles to radians
            FOV_rad = np.deg2rad(FOV)
            THETA_rad = np.deg2rad(THETA)
            PHI_rad = np.deg2rad(PHI)

            img_height, img_width = img.shape[:2]
            K = get_camera_matrix(FOV_rad, width, height)
            R = get_extrinsic_matrix(THETA_rad, PHI_rad)

            # Image grid
            x, y = np.meshgrid(np.arange(width), np.arange(height))
            z = np.ones_like(x)
            xyz = np.stack([x, y, z], axis=-1)

            world_coords = camera_to_world(xyz, K, R)
            sp_coords = cartesian_to_spherical(world_coords)
            XY = spherical2equirect(sp_coords, img_width, img_height)

            self._remap_cache[key] = XY  # cache for speed

        persp = cv2.remap(img, XY[..., 0], XY[..., 1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP)
        return persp

    def scan_qr_codes(self, frame):
        """Quét QR code từ ảnh equirectangular"""
        if not self.enabled:
            return []
        
        try:
            # --- Vòng 1: 6 view, zoom cao để quét QR gần ---
            views_round1 = [
                (0, 0),      # front
                (180, 0),    # back
                (90, 0),     # right
                (-90, 0),    # left
                (0, 90),     # up
                (0, -90),    # down
            ]
            zoom_round1 = [90, 70, 50]

            # --- Vòng 2: 12 view, zoom thấp để quét QR xa ---
            views_round2 = [
                (0, 0), (180, 0), (90, 0), (-90, 0),
                (0, 90), (0, -90),
                (45, 0), (-45, 0), (0, 45), (0, -45),
                (135, 0), (-135, 0)
            ]
            zoom_round2 = [30, 15, 7, 5]

            rounds = [
                (views_round1, zoom_round1),
                (views_round2, zoom_round2)
            ]

            all_results = []
            debug_view = None

            for views, zoom_levels in rounds:
                for fov in zoom_levels:
                    if all_results:  # dừng nếu đã quét QR
                        break

                    def process_view(args):
                        theta, phi = args
                        if cv2 is None:
                            return [], None
                        
                        try:
                            persp = self.Equirec2Perspec(frame, fov, theta, phi,
                                                        self.perspec_size, self.perspec_size)
                            gray = cv2.cvtColor(persp, cv2.COLOR_BGR2GRAY)
                            
                            # Sử dụng wrapper function để suppress warnings hoàn toàn
                            decoded_objects = _decode_qr_silently(gray)
                            
                            # Tất cả decoded_objects đã là QRCODE
                            qr_codes = decoded_objects
                            
                            results = []
                            debug_img = None

                            for qr in qr_codes:
                                results.append({
                                    "data": qr.data.decode('utf-8'),
                                    "rect": {
                                        "left": qr.rect.left,
                                        "top": qr.rect.top,
                                        "width": qr.rect.width,
                                        "height": qr.rect.height
                                    },
                                    "view": (theta, phi),
                                    "zoom": fov
                                })

                                # Tạo debug image nếu cần
                                x, y, w, h = qr.rect.left, qr.rect.top, qr.rect.width, qr.rect.height
                                debug_img = persp.copy()
                                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                                cv2.putText(debug_img, qr.data.decode('utf-8'), (x, y - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            return results, debug_img
                        except Exception as e:
                            # Tránh crash nếu có lỗi trong quá trình xử lý
                            return [], None

                    with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
                        futures = executor.map(process_view, views)
                        for r, dbg in futures:
                            all_results.extend(r)
                            if dbg is not None and debug_view is None:
                                debug_view = dbg

                if all_results:  # dừng vòng nếu đã quét QR
                    break

            return all_results

        except Exception as e:
            print(f'Error scanning QR codes: {str(e)}')
            import traceback
            traceback.print_exc()
            return []


if ROS2_AVAILABLE:
    class QRImageSubscriberNode(Node):
        """ROS2 Node để subscribe image topic và odometry cho QR scanning"""
        
        def __init__(self, image_topic, odom_topic, image_callback, odom_callback, node_name='qr_image_subscriber'):
            super().__init__(node_name)
            
            # Đảm bảo topic name có / prefix
            if not image_topic.startswith('/'):
                image_topic = '/' + image_topic
            if not odom_topic.startswith('/'):
                odom_topic = '/' + odom_topic
            
            # Subscribe image topic
            self.image_subscription = self.create_subscription(
                Image,
                image_topic,
                self.image_callback,
                10
            )
            
            # Subscribe odometry topic
            self.odom_subscription = self.create_subscription(
                Odometry,
                odom_topic,
                self.odom_callback,
                10
            )
            
            self.bridge = CvBridge()
            self.image_callback_func = image_callback
            self.odom_callback_func = odom_callback
            self.image_topic_name = image_topic
            self.odom_topic_name = odom_topic
            self.get_logger().info(f'QR Scanner subscribed to image topic {image_topic} and odom topic {odom_topic}')
        
        def image_callback(self, msg):
            """Callback when receiving image"""
            try:
                if cv2 is None or CvBridge is None:
                    return
                
                # Handle JPEG compressed images
                if msg.encoding.lower() in ['jpeg', 'jpg']:
                    # Decode JPEG data directly from compressed format
                    jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)
                    if cv_image is None:
                        raise ValueError("Failed to decode JPEG image")
                    # Convert BGR to RGB
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                else:
                    # Use cv_bridge for standard encodings
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                
                # Gọi callback với ảnh đã decode
                if self.image_callback_func:
                    self.image_callback_func(cv_image)
                
            except Exception as e:
                error_msg = f'Error processing image from {self.image_topic_name}: {e}'
                if hasattr(self, 'get_logger'):
                    self.get_logger().error(error_msg)
                print(f'[QRImageSubscriberNode] ERROR: {error_msg}')
        
        def odom_callback(self, msg):
            """Callback when receiving odometry"""
            try:
                # Extract position [x, y, z]
                position = [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]
                
                # Gọi callback với position
                if self.odom_callback_func:
                    self.odom_callback_func(position)
                
            except Exception as e:
                error_msg = f'Error processing odometry from {self.odom_topic_name}: {e}'
                self.get_logger().error(error_msg)
                print(f'[QRImageSubscriberNode] ERROR: {error_msg}')
else:
    # Dummy class nếu ROS2 không available
    class QRImageSubscriberNode:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("ROS2 not available. QRImageSubscriberNode cannot be used.")


__all__ = ['QRScanner', 'QRImageSubscriberNode', 'PYZBAR_AVAILABLE', 'GEOMETRY_UTILS_AVAILABLE', 'ROS2_AVAILABLE']
