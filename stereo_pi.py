#!/usr/bin/env python

# picamera stereo ROS node using dual CSI Pi CS3 board
# modified from code by https://github.com/realizator/StereoPi-ROS-depth-map-test

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import rospy
from sensor_msgs.msg import CameraInfo, Image
import yaml
import io
import signal # for ctrl-C handling
import sys

class RaspicamInterface:

    def __init__(self, res_x, res_y, target_FPS):
        signal.signal(signal.SIGINT, self.__signal_handler)
        self.left_img_msg = Image()
        self.right_img_msg = Image()
        self.left_cam_info = CameraInfo()
        self.right_cam_info = CameraInfo()
        self.res_x = res_x
        self.res_y = res_y
        self.target_FPS = target_FPS
        self.camera = None
        self.imageBytes = 0
        self.left_img_pub = None
        self.right_img_pub = None
        self.stream = None
        self.__setup_camera()
        self.__set_up_publishers()
        self.__init_ros_msg()
        self.timer = None

    def __parse_calibration_yaml(self,cam_info, calib_file):
        with file(calib_file, 'r') as f:
            params = yaml.load(f)
        cam_info.height = params['image_height']
        cam_info.width = params['image_width']
        cam_info.distortion_model = params['distortion_model']
        cam_info.K = params['camera_matrix']['data']
        cam_info.D = params['distortion_coefficients']['data']
        cam_info.R = params['rectification_matrix']['data']
        cam_info.P = params['projection_matrix']['data']
        return cam_info

    def __setup_camera(self):
        # initialize the camera
        print("Init camera...")
        self.camera = PiCamera(stereo_mode = 'top-bottom',stereo_decimate=False)
        self.camera.resolution = (self.res_x, self.res_y*2) # top-bottom stereo
        self.camera.framerate = self.target_FPS
        # using several camera options can cause instability, hangs after a while
        self.camera.exposure_mode = 'antishake'
       
        self.stream = io.BytesIO()

    def __set_up_publishers(self):
        # ----------------------------------------------------------
        #setup the publishers
        print("init publishers")
        # queue_size should be roughly equal to FPS or that causes lag?
        self.left_img_pub = rospy.Publisher('stereo/right/image_raw', Image, queue_size=1)
        self.right_img_pub = rospy.Publisher('stereo/left/image_raw', Image, queue_size=1)

        self.left_cam_pub = rospy.Publisher('stereo/right/camera_info', CameraInfo, queue_size=1)
        self.right_cam_pub = rospy.Publisher('stereo/left/camera_info', CameraInfo, queue_size=1)

        rospy.init_node('stereo_pub')
        self.timer = rospy.Timer(rospy.Duration(0.033), self.__camera_info_pub_clbk)

    def __init_ros_msg(self):
        # init messages
        self.left_img_msg.height = self.res_y
        self.left_img_msg.width = self.res_x
        self.left_img_msg.step = self.res_x*3 # bytes per row: pixels * channels * bytes per channel (1 normally)
        self.left_img_msg.encoding = 'rgb8'
        self.left_img_msg.header.frame_id = 'stereo_camera' # TF frame

        self.right_img_msg.height = self.res_y
        self.right_img_msg.width = self.res_x
        self.right_img_msg.step = self.res_x*3
        self.right_img_msg.encoding = 'rgb8'
        self.right_img_msg.header.frame_id = 'stereo_camera'

        self.imageBytes = self.res_x*self.res_y*3

        # parse the left and right camera calibration yaml files
        self.left_cam_info = self.__parse_calibration_yaml(self.left_cam_info, '/home/pi/workspace/StereoPi-ROS-depth-map-test/left.yaml')
        self.right_cam_info = self.__parse_calibration_yaml(self.right_cam_info,'/home/pi/workspace/StereoPi-ROS-depth-map-test/right.yaml')
    # ---------------------------------------------------------------
    # this is supposed to shut down gracefully on CTRL-C but doesn't quite work:
    def __signal_handler(self, signal, frame):
        print('CTRL-C caught')
        print('closing camera')
        self.camera.close()
        time.sleep(1)
        print('camera closed')    
        sys.exit(0)
    
    def __camera_info_pub_clbk(self, event):
        self.left_cam_pub.publish(self.left_cam_info)
        self.right_cam_pub.publish(self.right_cam_info)    

    def loop(self):
        print("Setup done, entering main loop")
        framecount=0
        frametimer=time.time()
        toggle = True
        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.stream, format="rgb", use_video_port=True):
            framecount +=1
            
            stamp = rospy.Time.now()
            self.left_img_msg.header.stamp = stamp
            self.right_img_msg.header.stamp = stamp
            self.left_cam_info.header.stamp = stamp
            self.right_cam_info.header.stamp = stamp    
            
            frameBytes = self.stream.getvalue()    
            self.left_img_msg.data = frameBytes[:self.imageBytes]
            self.right_img_msg.data = frameBytes[self.imageBytes:]      

            #publish the image pair
            self.left_img_pub.publish(self.left_img_msg)
            self.right_img_pub.publish(self.right_img_msg)
            
            # console info
            if time.time() > frametimer +1.0:
                if toggle: 
                    indicator = '  o' # just so it's obviously alive if values aren't changing
                else:
                    indicator = '  -'
                toggle = not toggle        
                print('approx publish rate:', framecount, 'target FPS:', self.target_FPS, indicator)
                frametimer=time.time()
                framecount=0
                
            # clear the stream ready for next frame
            self.stream.truncate(0)
            self.stream.seek(0)

if __name__ == '__main__':
    # cam resolution
    res_x = 320 #320 # per camera
    res_y = 240 #240 
    target_FPS = 15
    camera = RaspicamInterface(res_x, res_y, target_FPS)
    camera.loop()