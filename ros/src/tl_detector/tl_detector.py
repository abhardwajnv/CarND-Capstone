#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from collections import OrderedDict
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
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
        self.config = yaml.load(config_string)
        self.stop_line_positions = dict.fromkeys(list(tuple(x) for x in self.config['stop_line_positions']),None)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        for stop_light in self.stop_line_positions:
            try:
                #convert stop_light x,y position into pose type
                temp_pose = Pose()
                temp_pose.position.x = stop_light[0]
                temp_pose.position.y = stop_light[1]
                index = self.get_closest_waypoint(temp_pose)
                self.stop_line_positions[stop_light] = index
                rospy.logdebug("Stop_light_position = %s, closest_waypoint index = %s, and data = %s\n"%(stop_light,index, self.base_waypoints[index]))
            except Exception as E:
                print (E)
        self.stop_line_positions = {v: k for k, v in self.stop_line_positions.iteritems()}
        self.stop_line_positions = OrderedDict(sorted(self.stop_line_positions.items(), key=lambda s:s[0]))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def distance_between_coordinates(self,x1,y1,x2,y2):
        return math.sqrt(
                    (x1 - x2)**2+\
                    (y1 - y2)**2
                )

    def distance_between(self,pose1, pose2):
        return self.distance_between_coordinates(pose1.position.x, pose1.position.y, pose2.position.x, pose2.position.y)

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        current_minimum = 99999999999999999999999999999999
        waypoint_index = -1
        for i,waypoint in enumerate(self.base_waypoints):
            distance = self.distance_between(waypoint.pose.pose,pose)
            if (distance < current_minimum):
                current_minimum = distance
                waypoint_index = i
        '''
        # this was required in the way point updater, should not be required here
	if self._way_point_behind_car(waypoint_index):
            #rospy.logwarn("Waypoint %s behind the currept pose, picking next waypoint"%waypoint_index)
            waypoint_index += 1
        '''
        #rospy.logwarn("Returning waypoint index : %s"%waypoint_index)
        return waypoint_index

    def get_next_traffic_waypoint(self, car_waypoint_index):
        for key in self.stop_line_positions:
            rospy.logwarn("key = %s"%key)
            if key > car_waypoint_index:
                return key
        return None

    def light_is_close(self, light_wp_index, car_wp_index):
        signal_observe_value = 125
        x1 = self.base_waypoints[light_wp_index].pose.pose.position.x
        y1 = self.base_waypoints[light_wp_index].pose.pose.position.y
        x2 = self.base_waypoints[car_wp_index].pose.pose.position.x
        y2 = self.base_waypoints[car_wp_index].pose.pose.position.y
        distance = self.distance_between_coordinates(x1,y1,x2,y2) 
        rospy.logdebug("Car at : %s,%s traffic light at : %s,%s distance = %s\n"%(x2,y2,x1,y1,distance))
        if distance > signal_observe_value:
            rospy.logwarn("Car further than %s than next signal")
            return False
        else:
            rospy.logwarn("Car closer than %s than next signal, need to analyze the image from camera")
            return True

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # List of positions that correspond to the line to stop in front of for a given intersection
        if(self.pose and self.base_waypoints):
            car_wp_index = self.get_closest_waypoint(self.pose)
            light_wp_index = self.get_next_traffic_waypoint(car_wp_index)
            if self.light_is_close(light_wp_index, car_wp_index):
                #state = self.get_light_state()
                #return light_wp, state
                pass
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
