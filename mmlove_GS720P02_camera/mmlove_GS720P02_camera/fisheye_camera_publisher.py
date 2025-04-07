#########################
# Author: Mohandeep Kapur
#########################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2.typing import MatLike
import cv2
from std_msgs.msg import Header
from rclpy.clock import Clock
import subprocess

class FisheyeCameraPublisher(Node):
    """
    Interfaces with MMLove's 720P02 Fisheye Camera and publishes a stream of sensor_msgs/Image.
    """
    
    def __init__(self):        
        super().__init__(f'fisheye_camera_publisher')
        
        # cvbridge allows for conversion between cv::Mat to ROS2 sensor_msgs/Image type
        self.bridge = CvBridge()

        # extract device number argument
        self.declare_parameter('dev_num', value=-1)
        self.dev_num = self.get_parameter('dev_num').get_parameter_value().integer_value
       
        # check whether device number is valid
        if self.dev_num < 0:
            print('Invalid/No device number provided...')
            exit(1)

        # create image_raw publisher
        self.img_publisher = self.create_publisher(Image, f'image_raw', 10)

        # set callback rate 
        self.timer = self.create_timer(.03, self.publish_frame)

        self.init_video_capture_cpu(self.dev_num)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera...')
            exit(1)

        print(f'Publishing...')

    def publish_frame(self):
        # grabs, decodes, and returns the next video frame
        # RATE LIMITING CALLBACK - can't force callback rate past 30Hz when using CPU for decoding
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame...')
            return
        # optional: apply fisheye undistortion or preprocessing here
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # add timestamp + frame id
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = "fisheye_optical_frame"

        self.img_publisher.publish(msg)

    def undistort_frame(self, img : MatLike) -> MatLike:
        pass

    def init_video_capture_cpu(self, dev_num : int):
        # opens a camera for video capturing
        # v4l2-ctl --device=/dev/video2 --all
        # ^ can check whether device can be comm with using V4L2 API.
        print(f'Opening camera at /dev/video{dev_num}...')

        self.cap = cv2.VideoCapture(dev_num, cv2.CAP_V4L2)
        # setting camera video format to MJPG
        # to avoid USB bandwidth saturation
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # optional - set resolution and FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # fps set through cv is ok now
        self.cap.set(cv2.CAP_PROP_FPS, 8)

        self.set_exposure(dev_path=f"/dev/video{self.dev_num}", exposure=300)

        # check camera params - changing state of field doesn't mean hardware
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
        print(f"FPS: {fps}")
        print(f"Resolution: {int(width)}x{int(height)}")
        print("FOURCC format in use:", fourcc_str)
        

    # not working as of right now - opencv not built with gstreamer backend support
    def init_video_capture_gpu(self, dev_num : int):
        gst_str = (
            f'v4l2src device=/dev/video{dev_num} ! '
            f'image/jpeg, width=640, height=480, framerate=15/1 ! '
            f'nvjpegdec ! '
            f'nvvidconv ! '
            f'video/x-raw, format=BGRx ! '
            f'videoconvert ! '  # needed to convert from BGRx to BGR for OpenCV
            f'video/x-raw, format=BGR ! '
            f'appsink'
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    
    # def set_focus(self, focus_value):
    #     FOCUS_PROPERTY = 28
    #     if not self.cap.set(FOCUS_PROPERTY, focus_value):
    #         self.get_logger().error("Failed to set focus. Manual/auto focus might be locked.")
    #     else:
    #         self.get_logger().info(f"Focus set to {focus_value}")
    
    def set_exposure(self, dev_path, exposure=156):
        try:
            subprocess.run(["v4l2-ctl", "-d", dev_path, "-c", "auto_exposure=3"], check=True)
            #subprocess.run(["v4l2-ctl", "-d", dev_path, "-c", f"exposure_time_absolute={exposure}"], check=True)
            self.get_logger().info(f"Exposure set to {exposure}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to set exposure: {e}")



    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


################# Debugging Comments #################
# ros2 run mmlove_GS720P02_camera fisheye_camera_publisher --ros-args -p dev_num:=0
# ros2 launch mmlove_GS720P02_camera fisheye_quad.launch.py dev_nums:="[0, 2, 4, 6]"
# top
# sudo dmesg -w | grep -i usb
# gst-launch-1.0 v4l2src device=/dev/video4 ! image/jpeg, width=640, height=480, framerate=15/1 ! jpegdec ! autovideosink
# ^ fails at third device
# ls -l /dev/video*
# 1) usb bandwidth issue
# 2) callback rate limited by .read()

# lebron@lebron-desktop:~/ros2_ws$ v4l2-ctl -d /dev/video0 --get-fmt-video
# Format Video Capture:
#         Width/Height      : 640/480
#         Pixel Format      : 'MJPG' (Motion-JPEG)
#         Field             : None
#         Bytes per Line    : 0
#         Size Image        : 614400
#         Colorspace        : sRGB
#         Transfer Function : Rec. 709
#         YCbCr/HSV Encoding: ITU-R 601
#         Quantization      : Default (maps to Full Range)
#         Flags             : 

# 614kbytes per frame

# lebron@lebron-desktop:~/ros2_ws$ sudo usbtop
# [sudo] password for lebron: 
# No USB bus can be captured thanks to libpcap. Check your name filter and make sure relevent permissions are set !
# ^fw/kernel limit

# [ 2130.357360] input: Global Shutter Camera: Global S as /devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.4/1-2.4:1.0/input/input15
# [ 2146.284539] usb 1-2.3: new high-speed USB device number 17 using tegra-xusb
# [ 2146.464096] usb 1-2.3: Found UVC 1.00 device Global Shutter Camera (32e4:0144)
# [ 2146.545674] input: Global Shutter Camera: Global S as /devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.3/1-2.3:1.0/input/input16
# [ 2466.289540] usb 1-2.1: USB disconnect, device number 15
# [ 6057.775441] usb 1-2.4: Not enough bandwidth for new device state.
# [ 6057.775504] usb 1-2.4: Not enough bandwidth for altsetting 11
# [ 6325.692143] usb 1-2.3: USB disconnect, device number 17
# [ 6341.183563] usb 1-2.2: USB disconnect, device number 14
# [ 6351.728170] usb 1-2.4: USB disconnect, device number 16
# [ 6386.783595] usb 1-2.3: new high-speed USB device number 18 using tegra-xusb
# [ 6386.960971] usb 1-2.3: Found UVC 1.00 device Global Shutter Camera (32e4:0144)
# [ 6387.040398] input: Global Shutter Camera: Global S as /devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.3/1-2.3:1.0/input/input17
# [ 6409.582724] usb 1-2.1: new high-speed USB device number 19 using tegra-xusb
# [ 6409.759898] usb 1-2.1: Found UVC 1.00 device Global Shutter Camera (32e4:0144)
# [ 6409.841052] input: Global Shutter Camera: Global S as /devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.1/1-2.1:1.0/input/input18

# ^ i wouldn't trust these messages, because they're produced as soon as each camera is connected
# and the USB controller (or whateber) is coming to this conclusion looking at the max res/ lowest compression state
# of the camera, before i change that state in my ros2 nodes, decreasing max image size by 3x


# frames dropped -> buffer overload -> CPU overused or something?

'''
i wish i had a way to spin up diff instances of the same node, with that node publishing to diff variations
of the same topics (or same) easily (no need to hardcode this feature into the node's code)

solution: launch file
'''