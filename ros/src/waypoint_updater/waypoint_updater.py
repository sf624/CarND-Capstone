#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import std_msgs.msg
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
#import tf
import numpy as np

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = None
        self.previous_initial_wp_index = None
        self.traffic = None
        self.current_velocity = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(25) # 25 Hz is half of the twist controller
        is_lost = True
        while not rospy.is_shutdown():
            rate.sleep()
            if is_lost:  # check if all is initialized and localize the car 
                if self.pose is not None and self.waypoints is not None and self.traffic is not None and self.current_velocity is not None:
                    # Search the closer waypoint
                    self.previous_initial_wp_index = self.global_waypoint_search()
                    is_lost = False 
                    continue
                else:
                    continue      
            

            #initial_wp_index = self.global_waypoint_search()
            [is_ok,initial_wp_index] = self.relative_waypoint_search(self.previous_initial_wp_index, 50) #window is 100 should be enugh at 25 m/s we do 2 waypoiny per step!!!
            if not is_ok: # we care lost compute global position
                self.previous_initial_wp_index = self.global_waypoint_search()                
                continue

            # publish
            final_waypoints = Lane()
            final_waypoints.header = std_msgs.msg.Header()
            final_waypoints.header.stamp = rospy.Time.now()
            for i in xrange(LOOKAHEAD_WPS):
                index = (initial_wp_index + i) % len(self.waypoints)
                final_waypoints.waypoints.append(self.waypoints[index])

            self.previous_initial_wp_index = initial_wp_index

            v_limit = rospy.get_param('/waypoint_loader/velocity') / 3.6  # Speed limit given by ROS parameter
            v_limit *= 0.9           # Set margin to not exceed speed limit.
            v0 = v_limit; # min(25.0, v_limit)  # This program allows maximum spped of 25m/s.

            if self.traffic == -1:
                for i in xrange(LOOKAHEAD_WPS):
                    self.set_waypoint_velocity(final_waypoints.waypoints, i, v0)
            else:
                    #                    t0
                    #   v0 ----------------
                    #                      \
                    #                       \
                    #                        \ a0
                    #                         \
                    #                          \
                    #                           \___________ v=0
                    #                           t1
                    #     target velocity diagram
                    #
                a0 = 2.5        # m/s^2  target acceleration
                margin = 10      # m      target margin before stop line
                r0 = self.distance(self.waypoints,initial_wp_index,self.traffic) - margin  # target position to stop
                t1 = 0.5*(2.*r0/v0 + v0/a0)
                t0 = 0.5*(2.*r0/v0 - v0/a0)

                for i in xrange(LOOKAHEAD_WPS):
                    r = self.distance(self.waypoints,initial_wp_index,initial_wp_index+i)
                    if r <= v0 * t0:
                        v = v0
                    elif v0*t0 < r and r < r0:
                        v = math.sqrt(2.*a0*v0*t0 + v0*v0 - 2.*a0*r)
                    else:
                        v = -1
                    self.set_waypoint_velocity(final_waypoints.waypoints, i, v)
            self.final_waypoints_pub.publish(final_waypoints)

                

    # relative search of the waypoint, in a window centered in center with size window_size
    def relative_waypoint_search(self,center, window_size, distance_threshold=20):
        min_distance = sys.float_info.max
        ref_wp_orient = self.waypoints[center].pose.pose.orientation
        [yaw,pich,roll] = self.quaternion_to_euler_angle(ref_wp_orient)
        ego_position = np.array([self.pose.position.x,self.pose.position.y])
        for rel_index in range(2*window_size):    
            k = (rel_index - window_size + center) % len(self.waypoints)
            wp = self.waypoints[k].pose.pose.position
            wp_position = np.array([wp.x,wp.y])
            # Find the vector that joing the ego positon with the position of the way point 
            join_vect = wp_position - ego_position
            ego_vx_versor = np.array([math.cos(roll),math.sin(roll)])
            distance  = np.linalg.norm(join_vect)
            param = join_vect[0]*ego_vx_versor[0] + join_vect[1]*ego_vx_versor[1]#np.dot(ego_vx_versor, join_vect)
            # If the waypoint is in front of vehicle and also the closest one,
            # update the index.
            #param = 1 
            if(param > 0 and distance < min_distance):
                min_distance = distance
                initial_wp_index = k
        if min_distance>distance_threshold:
            rospy.loginfo('WaypUPD  car is lost, waypoint distance {0} [m] '.format(min_distance))
            return False, center
        rospy.loginfo('WaypUPD   waypoints number {0}  relative'.format(initial_wp_index))
        return True, initial_wp_index  

    # compute out of waypoints list the closer waypoint (in front of the car) to the current pose of the robot        
    def global_waypoint_search(self):
        min_distance = sys.float_info.max
        #[yaw,pich,roll] = self.quaternion_to_euler_angle(self.pose.orientation)
        ego_position = np.array([self.pose.position.x,self.pose.position.y])
        initial_wp_index = -1
        for k in xrange(len(self.waypoints)):    # TODO: you can do a rough search here
            wp = self.waypoints[k].pose.pose.position
            wp_position = np.array([wp.x,wp.y])
            # Find the vector that joing the ego positon with the position of the way point 
            join_vect = wp_position - ego_position
            #ego_vx_versor = np.array([math.cos(roll),math.sin(roll)])
            distance  = np.linalg.norm(join_vect)
            #param = np.dot(ego_vx_versor, join_vect)
            # If the waypoint is in front of vehicle and also the closest one,
            # update the index.
            #param = 1 
            #if(param > 0 and distance < min_distance):
            
            if(distance < min_distance):  # find the closest point to the vehicle
                min_distance = distance
                initial_wp_index = k
                
        rospy.loginfo('WaypUPD waypoints number {0}  global'.format(initial_wp_index))
        return initial_wp_index


    # Conversion quaternion to euler angle
    @staticmethod
    def quaternion_to_euler_angle(orientation):
        w = orientation.w
        x = orientation.x
        y = orientation.y
        z = orientation.z
        #rospy.loginfo('WaypUPDD quaternion z {0} '.format(z))
	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z

    def current_velocity_cb(self,msg):
        self.current_velocity = msg.twist

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        pass

    def waypoints_cb(self, msg):
        # TODO: Implement
	rospy.loginfo('waypoints callback'.format())
        self.waypoints = msg.waypoints
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
