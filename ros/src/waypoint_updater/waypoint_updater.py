#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import tf
'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''
MAX_VELOCITY = 11
MAX_ACCELERATION = 9
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater',log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.base_waypoint_sub = rospy.Subscriber("/base_waypoints", Lane, self.waypoints_cb)
        self.traffic_waypoint_sub = rospy.Subscriber("/traffic_waypoint", Int32 , self.traffic_cb)
        self.current_pose_sub = rospy.Subscriber("/current_pose", PoseStamped,self.pose_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.current_velocity = -1
        self.stop_waypoint = -1
        self.base_waypoints = None
        self.pose = None
        self.final_waypoints_seq = 0
        self.final_waypoints_frame_id = None
        self.rate = rospy.Rate(50)
        self.update_waypoints()

    @property
    def current_euler_position(self):
        return tf.transformations.euler_from_quaternion((
                                                        self.pose.orientation.x,\
                                                        self.pose.orientation.y,\
                                                        self.pose.orientation.z,\
                                                        self.pose.orientation.w)
                                                       )



    @property
    def current_yaw(self):
        return self.current_euler_position[2]

    def current_velocity_cb(self,msg):
        if msg.twist.linear.x < 0.0001:
            self.current_velocity = 0.0
        else:
            self.current_velocity = msg.twist.linear.x

    def _get_distance_from_current_pose(self,pose):
        return self.distance_between(self.pose, pose)

    def _way_point_behind_car(self,waypoint_index):
        #TODO : I am pretty sure this is the wrong way, need to find out which is a correct way, and use the same
        #x_car_system = ((map_x-x) * math.cos(yaw) + (map_y-y) * math.sin(yaw))
        yaw = self.current_yaw
        x_car_reference = (
                           (self.base_waypoints[waypoint_index].pose.pose.position.x - self.pose.position.x) * math.cos(yaw)+
                           (self.base_waypoints[waypoint_index].pose.pose.position.y - self.pose.position.y) * math.sin(yaw)
                          )

        #rospy.logwarn("Car_reference x = %s "%x_car_reference)
        return x_car_reference < 0.0;

    def _get_closest_waypoint(self):
        current_minimum = 99999999999999999999999999999999
        waypoint_index = -1
        for i,waypoint in enumerate(self.base_waypoints):
            distance = self._get_distance_from_current_pose(waypoint.pose.pose)
            if (distance < current_minimum):
                current_minimum = distance
                waypoint_index = i
        if self._way_point_behind_car(waypoint_index):
            #rospy.logwarn("Waypoint %s behind the currept pose, picking next waypoint"%waypoint_index)
            waypoint_index += 1
        #rospy.logwarn("Returning waypoint index : %s"%waypoint_index)
        return waypoint_index

    @property
    def deccelration_distance(self):
        return MAX_VELOCITY**2/(2*MAX_ACCELERATION)
   
    def brake(self, car_waypoint_index):
        #if self.current_velocity == 0:
        #    return []
        temp_waypoint = []
        distance_to_stoppoint = self.distance_between(self.base_waypoints[car_waypoint_index].pose.pose,
                                                      self.base_waypoints[self.stop_waypoint].pose.pose)
        if distance_to_stoppoint <= 1:
            return []
        a = self.current_velocity**2/(2*distance_to_stoppoint)
        if a > MAX_ACCELERATION:
            a = MAX_ACCELERATION
        a = a*-1
        for way_point in self.base_waypoints[car_waypoint_index:car_waypoint_index+LOOKAHEAD_WPS]:
            distance_to_waypoint = self.distance_between(self.base_waypoints[car_waypoint_index].pose.pose,
                                                         way_point.pose.pose)
            if distance_to_waypoint == 0:
                speed = 0
            else:
                #speed = math.sqrt(self.current_velocity**2 + 2*MAX_ACCELERATION*distance_to_waypoint*-1)
                speed = math.sqrt(self.current_velocity**2 + 2*a*distance_to_waypoint)
            #rospy.logwarn("car_distance_to_waypoint = %s, speed = %s, car_distance_to_stop_point = %s"%(distance_to_waypoint, speed, distance_to_stoppoint))
            self.set_waypoint_velocity(way_point,speed)
            temp_waypoint.append(way_point)
            if distance_to_waypoint == 0:
                break
        return temp_waypoint

    def accelerate(self,car_waypoint_index):
        #rospy.logwarn("Accelerating")
        temp_waypoint = []
        distance_to_stoppoint = self.distance_between(self.base_waypoints[car_waypoint_index].pose.pose,
                                                      self.base_waypoints[self.stop_waypoint].pose.pose)
        for way_point in self.base_waypoints[car_waypoint_index:car_waypoint_index+LOOKAHEAD_WPS]:
            distance_to_waypoint = self.distance_between(self.base_waypoints[car_waypoint_index].pose.pose,
                                                         way_point.pose.pose)
            speed = MAX_VELOCITY
            #rospy.logwarn("car_distance_to_waypoint = %s, speed = %s, car_distance_to_stop_point = %s"%(distance_to_waypoint, speed, distance_to_stoppoint))
            self.set_waypoint_velocity(way_point,speed)
            temp_waypoint.append(way_point)
        return temp_waypoint

    def update_waypoint_list(self, car_waypoint_index):
        OFFSET = 10
        if self.stop_waypoint != -1:
            distance_to_stop_point = self.distance_between(self.base_waypoints[car_waypoint_index].pose.pose,\
                                                           self.base_waypoints[self.stop_waypoint].pose.pose)
            rospy.logwarn("Distance to stop point = %s, decceleration_distance = %s"%(distance_to_stop_point, self.deccelration_distance))
            if distance_to_stop_point < self.deccelration_distance + OFFSET:
                way_point_list = self.brake(car_waypoint_index)
            else:
                way_point_list = self.accelerate(car_waypoint_index)
        else:
            #rospy.logwarn("Not Red signal")
            way_point_list = self.accelerate(car_waypoint_index)
            #accelerate to max_velocity
        return way_point_list

    def update_waypoints(self):
        while not rospy.is_shutdown():
            if self.base_waypoints is None or self.pose is None:
                #rospy.logwarn("Cannot publish next waypoints, base_waypoint = %s, current_pose = %s\r"%(self.base_waypoints, self.pose))
                #rospy.logwarn("Cannot publish next waypoints, current pose or base_waypoint not available\r")
                pass
            else:
                rospy.logdebug("Attempting to publish final way points")
                car_waypoint_index = self._get_closest_waypoint()
                way_point_list = self.update_waypoint_list(car_waypoint_index)

                final_waypoints_message = Lane()
                final_waypoints_message.header.seq = self.final_waypoints_seq
                final_waypoints_message.header.frame_id = self.final_waypoints_frame_id
                final_waypoints_message.waypoints.extend(way_point_list)
                self.final_waypoints_pub.publish(final_waypoints_message)
            self.rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.logdebug("Got current position")
        self.pose = msg.pose
        #This frame id and sequence is added to the final waypoint head, need to check if this value is what is expected
        self.final_waypoints_frame_id = msg.header.frame_id
        self.final_waypoints_seq = msg.header.seq

    def waypoints_cb(self, msg):
        # TODO: Implement
        rospy.logdebug("Got base waypoints")
        self.base_waypoints = msg.waypoints
        for way_point in self.base_waypoints:
            self.set_waypoint_velocity(way_point ,0)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance_between(self,pose1, pose2):
        return math.sqrt(
                            (pose1.position.x - pose2.position.x)**2+
                            (pose1.position.y - pose2.position.y)**2
                            #+(wp1.pose.pose.position.z - wp2.pose.pose.position.z)**2+
                        ) 

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
