#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None

        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        
        config_string = rospy.get_param("/traffic_light_config")
        #config_string = rospy.get_param("sim_traffic_light_config.yaml")
        self.config = yaml.load(config_string)
        #self.config = yaml.load("sim_traffic_light_config.yaml")
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.st_count = 0
        
        self.waypoints_2d = None
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def traffic_cb(self, msg):
        self.lights = msg.lights
        

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        #return 0
        index = self.waypoint_tree.query([x,y], 1)[1]
        
        closest_coord = self.waypoints_2d[index]
        prev_coord = self.waypoints_2d[index - 1]
        
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        
        if 0 < val:
            index = (index + 1) % len(self.waypoints_2d)
            
        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #if(not self.has_image):
        #    self.prev_light_loc = None
        #    return False
        #
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        (H, W) = cv_image.shape[:2]
        #area
        area = cv_image[int(0.0*H):int(0.8*H), int(0.2*W):int(0.8*W)]
        hsv = cv2.cvtColor(area, cv2.COLOR_BGR2HSV)
        #color
        """
        # red value1
        hsv_min = np.array([0,32,0])
        hsv_max = np.array([30,255,255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # red value2
        hsv_min = np.array([150,32,0])
        hsv_max = np.array([179,255,255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # mask area 255:red 0:others  
        mask = mask1 + mask2
        #mask = cv2.bitwise_and(mask1, mask2)
        #mask = cv2.inRange(hsv, hsv_min, hsv_max)
        """
        
        
        hsv_min = np.array([150,200,50])
        hsv_max = np.array([179,255,255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)     
        hsv_min = np.array([0,200,50])
        hsv_max = np.array([30,255,255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = mask1 + mask2
        
        mask_value = cv2.countNonZero(mask)
        
        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        
        red_value = 300
        yellow_value = int(red_value*0.5)
        
        if red_value < mask_value:
            signal = TrafficLight.RED
        elif yellow_value < mask_value:
            signal = TrafficLight.YELLOW
        else:
            signal = TrafficLight.GREEN
        rospy.logwarn("Detected Light: mask={}, signal={}".format(mask_value, signal))
        #return light.state
        return signal

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light_traffic = None
        light_waypoint_idx =None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if self.pose and self.waypoint_tree is not None:
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
        
            for idx, light in enumerate(self.lights):
                waypoint_idx =self.get_closest_waypoint(stop_line_positions[idx][0], stop_line_positions[idx][1])
                waypoint_idx_diff = waypoint_idx - car_position
                
                if waypoint_idx_diff >= 0 and waypoint_idx_diff < diff:
                    diff = waypoint_idx_diff
                    closest_light_traffic = light
                    light_waypoint_idx = waypoint_idx

        if closest_light_traffic:
            state = self.get_light_state(closest_light_traffic)
            return light_waypoint_idx, state
            
        else:
            light_waypoint_idx = -1
            state = TrafficLight.UNKNOWN
            return -1, state
            
        #rospy.logwarn("Detected Light: line_waypoint_indx={}, state={}".format(light_waypoint_idx, self.to_string(state)))
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN
        
        #return light_waypoint_idx, state
        
    def to_string(self, state):
        out = "unknown"
        if state == TrafficLight.GREEN:
            out = "green"
        elif state == TrafficLight.YELLOW:
            out = "yellow"
        elif state == TrafficLight.RED:
            out = "red"
        return out
        
        
    ############################
    #####Call back SETTINGS#####
    ############################
    def pose_cb(self, msg):
        self.pose = msg
        
    def waypoints_cb(self, waypoints):
        self.waypoints=waypoints
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            
    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.logwarn("tl_detector: traffic_cb={}".format(self.lights))
        
    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        
        if self.state != state:
            self.st_count = 0
            self.state = state
            
        elif self.st_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            
        self.st_count += 1
    
    
    ############################
    #####Call back SETTINGS#####
    ############################
        
        
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
