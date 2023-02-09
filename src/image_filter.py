#!/usr/bin/env python

# Import modules
import rospy
import cv2
import message_filters
import numpy as np

# Import message types and other pyton libraries
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2

class ImageFilter():
    """
    A class that filters the head camera image and publishes a filtered image
    and depth image.
    """
    def __init__(self):
        """
        A Function that initializes the subscribers, variables, and publisher.
        :param self: The self reference.
        """
        ## Initialize publishers
        self.publisher       = rospy.Publisher('filtered_image',       Image, queue_size=5)
        self.depth_publisher = rospy.Publisher('filtered_depth_image', Image, queue_size=5)

        ## Initialize subscribers
        self.image_sub       = message_filters.Subscriber('head_camera/rgb/image_raw',          Image)
        self.depth_image_sub = message_filters.Subscriber('head_camera/depth_registered/image', Image)

        sync = message_filters.ApproximateTimeSynchronizer([self.image_sub,
                                                            self.depth_image_sub],
                                                            queue_size=5,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)

        ## set the lower and upper bounds for the desired hue
        self.lower_col = np.array([0, 180, 225])
        self.upper_col = np.array([174, 255, 255])

        #self.lower_col = np.array([10, 30, 30])
        #self.upper_col = np.array([255, 255, 255])

        ## Instantiate a `CvBridge` object
        self.bridge = CvBridge()

        ## Notify user that node is up and running
        rospy.loginfo("image_filter node running. Check rviz to view filtered image")

    def callback_sync(self, img_msg, depth_msg):
        """
        Function that sync two image message types, creates a mask and filters both
        messages.
        :param self: The self reference.
        :param img_msg: A Image message type.
        :param depth_msg: A Image message type.
        """
        ## INFO:If the colors are weird, this may need to be changed to BGR
        ##      instead of RGB. There's only a handful of instances of this, so
        ##      you should be able to find them without too much trouble with ctrl-F

        ## Convert ROS image type to OpenCV message type. Then convert the image
        ## color spcace to another by using the `cv2.cvtColor()` method
        image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(img_msg, 'rgb8'), cv2.COLOR_RGB2HSV)

        ## Create a mask for violet color using the inRange method
        mask = cv2.inRange(image, self.lower_col, self.upper_col)

        ## Perform bitwise on the original image arrays using the mask
        result = cv2.bitwise_and(image, image, mask=mask)

        ## Convert OpenCV message type to a ROS image
        filtered_image = self.bridge.cv2_to_imgmsg(cv2.cvtColor(result, cv2.COLOR_HSV2RGB), 'rgb8')

        ## Update the img_msg with the filtered data. Then publish that image
        img_msg.data = filtered_image.data
        self.publisher.publish(img_msg)

        ## Convert ROS image type to OpenCV message type
        depth_cv2 = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        ## Perform bitwise on the depthmap image arrays using the mask
        depth_filtered = cv2.bitwise_and(depth_cv2, depth_cv2, mask=mask)

        ## Convert OpenCV message type to a ROS image
        filtered_depth_image = self.bridge.cv2_to_imgmsg(depth_filtered, '16UC1')

        ## Update the depth_msg with the filtered depth image data. Then publish
        depth_msg.data = filtered_depth_image.data
        self.depth_publisher.publish(depth_msg)

if __name__=="__main__":
    ## Initialize the `image_filter` node
    rospy.init_node("image_filter", anonymous=True)

    ## Instantiate a `ImageFilter` object
    filter = ImageFilter()
    rospy.spin()
