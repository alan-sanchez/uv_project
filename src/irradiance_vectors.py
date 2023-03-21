#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import tf

# Import message types and other python libraries
from best_fit import fit
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class IrradianceVectors(object):
    """
    A class that publishes the direction and irridance values of the UV vectors
    whose origins are from the end effector.
    """
    def __init__(self):
        """
        A function that initialises  the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscribers
        self.start_sub = rospy.Subscriber('start', String, self.ir_vec_calculator)
        self.stop_sub  = rospy.Subscriber('stop',  String, self.callback_stop_command)

        ## Initialize Publishers
        self.vector_array_pub = rospy.Publisher('vectors', numpy_msg(Floats), queue_size=10)
        # self.ee_pose_pub      = rospy.Publisher('ee_pose', PoseStamped,       queue_size=10)

        ## Initialize transform listener
        self.listener = tf.TransformListener()

        ## Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        ## Initialize the end effector PoseStamped message type. Used for data storage
        self.ee_pose_stamped = PoseStamped()
        self.ee_pose_stamped.header = self.header

        ## Bring in the UV light source model as an equation
        self.eqn_model = fit(16, plotter = False)

    def callback_stop_command(self, msg):
        """
        A callback function that stores a String message that stops this node from
        publishing irridiance vectors.
        :param self: The self reference.
        :param msg: The String message type.
        """
        self.command = msg.data

    def ir_vec_calculator(self, msg):
        """
        A callback function that generates vectors with their given irradiance values
        and publishes those values, along with the end effector location.
        :param self: The self reference.
        :param msg: The String message type.
        """
        ## Set self.command to incoming String message
        self.command = msg.data

        ## Generate 3 circle layers of points that will serve as the LEADING points of
        ## the UV castRays. there will be 1 point in the center, 6 in the middle layer
        ## and 12 in the outer layer. These values are defined in a list named "n".
        ## "r" reresents the layer distance from the center point in centimeters
        n = [1, 6, 12]
        r = [0.0, 2.5, 5.0]

        ## From the previous defined list, n and r, we generate the directional vectors 
        ## using the `self.circle_points` method
        dir_vectors = self.circle_points(r,n)

        ## Create an empty numpy array, size of a 19 by 4 matrix. The first three columns
        ## will be the x, y, and z vector directions, respectviely. The last column will be
        ## the irradiance value of that directional vector, which is acquired from the model
        self.vectors = np.empty(shape=[len(dir_vectors),4])

        ## Rospy rate
        rate = rospy.Rate(20.0)
        
        while self.command == "start":
            ## The `self.find_ee_pose()` method gets the translational and rotational 
            ## difference from the ee_link to the base_link. Essentially, the ee_link's
            ## coordinates in reference to the base_link transform frame
            ee_trans, ee_rot = self.find_ee_pose()

            i = 0

            ## For each directional vector, we include the irradiance value
            for coord in dir_vectors:

                ## Compute the direction vector from the coordinates of the
                ## transformed point and the end effector point
                self.vectors[i] = [coord[0],coord[1],coord[2],1]

                ## Conditional statement to fill in the irradiance values from our model.
                ## These values will be stored in the 4th column of the vectors dataset
                if i == 0:
                    self.vectors[i][3] = self.eqn_model(r[0])

                elif i > 0 and i < 6:
                    self.vectors[i][3] = self.eqn_model(r[1])

                elif i >= 6:
                    self.vectors[i][3] = self.eqn_model(r[2])

                ## Increment for the conditional statement above
                i+=1

            ## Publish both the location of the end effector and direction vectors
            ## in a single numpy array
            ee_pose = np.array([ee_trans[0],ee_trans[1],ee_trans[2]], dtype=np.float32)
            dir_ir_vector = np.array(self.vectors.ravel(), dtype=np.float32)
            self.vector_array_pub.publish(np.concatenate((ee_pose, dir_ir_vector)))

            ## Conditional statement to break while loop
            if self.command != "start":
                break

            rate.sleep()

    def find_ee_pose(self):
        """
        Function that finds the pose of the ee_link relative to the base_link frame
        :param self: The self reference.

        :return [trans, rot]: The Pose message type.
        """
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/gripper_link',rospy.Time(0))
                return [trans,rot]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def circle_points(self, r, n):
        """
        Function that generates the head positions of the CastRays.
        :param self: The self reference.
        :param r: Float value.
        :param n: Float value.

        :return concatenate: Python list.
        """
        circles = []
        for r, n in zip(r, n):
            t = np.linspace(0, 2*np.pi, n, endpoint=False)
            x = 10**-2*r * np.cos(t)    # Convert to meters by using 10**-2
            y = 10**-2*r * np.sin(t)    # Convert to meters by using 10**-2
            z = [-0.3]*len(x)
            circles.append(np.c_[x, y, z])
            concatenate = np.concatenate( circles, axis=0 )
        return concatenate

if __name__=="__main__":
    # Initialize irradiance_vectors node
    rospy.init_node('irradiance_vectors',anonymous=True)

    # Instantiate the IrradianceVectors class
    IrradianceVectors()
    rospy.spin()
