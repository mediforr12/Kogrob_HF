#!/usr/bin/env python3

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from tensorflow.compat.v1 import InteractiveSession
from tensorflow.compat.v1 import ConfigProto
from tensorflow.keras import __version__ as keras_version
import tensorflow as tf

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
import rospkg
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import h5py
import time

# Set image size
image_size = 24

# Initialize Tensorflow session
config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

# Initialize ROS node and get CNN model path
rospy.init_node('line_follower')

rospack = rospkg.RosPack()
path = rospack.get_path('turtlebot3_hf')
model_path = path + "/network_model/model.best.h5"

print("[INFO] Version:")
print("OpenCV version: %s" % cv2.__version__)
print("Tensorflow version: %s" % tf.__version__)
keras_version = str(keras_version).encode('utf8')
print("Keras version: %s" % keras_version)
print("CNN model: %s" % model_path)
f = h5py.File(model_path, mode='r')
model_version = f.attrs.get('keras_version')
print("Model's Keras version: %s" % model_version)

if model_version != keras_version:
    print('You are using Keras version ', keras_version, ', but the model was built using ', model_version)

# Finally load model:
model = load_model(model_path)

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue, position):
        threading.Thread.__init__(self)
        self.queue = queue
        self.image = None
        self.position = position

        # Initialize published Twist message
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.color = [0,0,0]
        self.last_time = time.time()

        self.bot_path=[]
        self.path_color=[]

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.queue.get()

            # Process the current image
            mask1 = self.processImage(self.image)
            mask2 = self.drawTrajectory(self.position, self.color)
            # Add processed images as small images on top of main image
            result = self.addSmallPictures(self.image, [mask1, mask2])
            cv2.imshow("frame", result)

            # Check for 'q' key to exit
            k = cv2.waitKey(1) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                pub.publish(self.cmd_vel)
                # Quit
                rospy.signal_shutdown('Quit')

    def processImage(self, img):

        image = cv2.resize(img, (image_size, image_size))
        image = img_to_array(image)
        image = np.array(image, dtype="float") / 255.0

        image = image.reshape(-1, image_size, image_size, 3)
        
        with tf.device('/gpu:0'):
            prediction = model(image, training=False)
            prediction_dir = np.argmax(prediction[0:4])
            prediction_color = np.argmax(prediction[4:10])
                
        print("Prediction %d, elapsed time %.3f" % (prediction_dir, time.time()-self.last_time))
        self.last_time = time.time()
        if prediction_color == 0: # Red
            self.color = [1,0,0]
            color_linfaktor = 0.5
            color_angfaktor = 1.5
        elif prediction_color == 1: # Green
            self.color = [0,1,0]
            color_linfaktor = 1.2
            color_angfaktor = 1
        elif prediction_color == 2: # Blue
            self.color = [0,0,1]
            color_linfaktor = 1
            color_angfaktor = 1
        elif prediction_color == 3: # Magenta
            self.color = [1,0,1]
            color_linfaktor = 2
            color_angfaktor = 0.5
        elif prediction_color == 4: # Yellow
            self.color = [1,1,0]
            color_linfaktor = 0.8
            color_angfaktor = 1.2
        elif prediction_color == 5: # Cian
            self.color = [0,1,1]
            color_linfaktor = 0.7
            color_angfaktor = 0.8
        if prediction_dir == 0: # Forward
            self.cmd_vel.angular.z = 0*color_angfaktor
            self.cmd_vel.linear.x = 0.1*color_linfaktor
        elif prediction_dir == 1: # Right
            self.cmd_vel.angular.z = -0.2*color_angfaktor
            self.cmd_vel.linear.x = 0.05*color_linfaktor
        elif prediction_dir == 2: # Left
            self.cmd_vel.angular.z = 0.2*color_angfaktor
            self.cmd_vel.linear.x = 0.05*color_linfaktor
        elif prediction_dir == 3: # Stop
            self.cmd_vel.angular.z = 0.1*color_angfaktor
            self.cmd_vel.linear.x = 0.0*color_linfaktor

        # Publish cmd_vel
        pub.publish(self.cmd_vel)
        
        # Return processed frames
        return cv2.resize(img, (image_size, image_size))

    # Add small images to the top row of the main image
    def addSmallPictures(self, img, small_images, size=(160, 120)):
        '''
        :param img: main image
        :param small_images: array of small images
        :param size: size of small images
        :return: overlayed image
        '''

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img
    
    def drawTrajectory(self, position, color):
        self.bot_path.append(position)
        self.path_color.append(color)
        fig = plt.figure()
        plot = fig.add_subplot(1,1,1)
        plot.plot([point[0] for point in self.bot_path],
                 [point[1] for point in self.bot_path],
                 color=self.path_color)
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        
        return cv2.resize(img,(image_size,image_size))


def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
        cv2Img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

def Position(odom_data):
    position_x=odom_data.pose.pose.position.x
    position_y=odom_data.pose.pose.position.y
    position[0], position[1] = position_x, position_y

queueSize = 1
position = [0,0]      
qMono = BufferQueue(queueSize)
bridge = CvBridge()


# Define your image topic
image_topic = "/camera/image/compressed"
# Set up your subscriber and define its callback
rospy.Subscriber(image_topic, CompressedImage, queueMonocular)
rospy.Subscriber('odom', Odometry,Position)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
# Start image processing thread
cvThreadHandle = cvThread(qMono,position)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

# Spin until Ctrl+C
rospy.spin()