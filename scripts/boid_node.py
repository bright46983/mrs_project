#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, OccupancyGrid
import sys
import numpy as np
import re
from boid import Boid
from visualization_msgs.msg import Marker, MarkerArray

class BoidNode:
    def __init__(self,id,boid_count):
        rospy.init_node('boid_node', anonymous=True)

        self.id = id
        self.frame_id = '/robot_{}/odom'.format(id)
        self.boid_count = boid_count
        

        self.enable_visualization = True
        self.visualize_array = MarkerArray()

        self.boid = Boid()
        self.vel_pub = rospy.Publisher('robot_{}/cmd_vel'.format(self.id), Twist, queue_size=10)
        self.visual_pub = rospy.Publisher('robot_{}/visualization'.format(self.id), MarkerArray, queue_size=10)
        
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('map', OccupancyGrid, self.map_cb)

        self.other_boids = []
        for i in range(boid_count):
            self.other_boids.append(Boid())
            rospy.Subscriber('robot_{}/odom'.format(i), Odometry, self.boid_cb)

        self.other_boids[self.id] = None

        self.dt = 0.1

        rospy.Timer(rospy.Duration(self.dt), self.run)
        rospy.Timer(rospy.Duration(self.dt), self.visualize)


    def boid_cb(self, msg):
        if msg.header.frame_id == self.frame_id:
            # update current state of the boid
            self.boid.position = msg.pose.pose.position
            self.boid.velocity.x = msg.twist.twist.linear.x
            self.boid.velocity.y = msg.twist.twist.linear.y
            self.boid.heading = np.arctan2(self.boid.velocity.y ,self.boid.velocity.x)

        else:
            id = int(re.findall(r'\d+', msg.header.frame_id)[0])
    
            self.other_boids[id].position = msg.pose.pose.position
            self.other_boids[id].velocity.x = msg.twist.twist.linear.x
            self.other_boids[id].velocity.y = msg.twist.twist.linear.y
            self.other_boids[id].heading = np.arctan2(self.other_boids[id].velocity.y, self.other_boids[id].velocity.x)

            

    def map_cb(self, msg):
        self.boid.map = msg
            

    def goal_cb(self, msg):
        self.boid.goal = msg.pose.position


    def run(self,_):
        self.boid.update_neigbors(self.other_boids)

        nav_acc = self.boid.navigation_acc()
        sep_acc = self.boid.seperation_acc()
        coh_acc = self.boid.cohesion_acc()
        align_acc = self.boid.allignment_acc()
        obs_acc = self.boid.obstacle_acc()
        
        all_acc = self.boid.combine_acc(nav_acc,sep_acc,coh_acc,align_acc,obs_acc)
        out_vel = self.boid.cal_velocity(all_acc,self.dt)

        cmd_vel = Twist()
        cmd_vel.linear.x = out_vel.x
        cmd_vel.linear.y = out_vel.y

        self.vel_pub.publish(cmd_vel)

    ################################################
    #### Visualization
    ################################################
    def visualize(self,_):
        if self.enable_visualization:
            self.visualize_neighbor()
            self.visualize_goal()
            self.visualize_acc()

            self.visual_pub.publish(self.visualize_array)

    def visualize_neighbor(self):
        marker_id  = 0
        scale = [0.15, 0.15, 0.15]
        color = [1,0,0,1]
        frame_id = "map"
        ns = "neighbors"

        marker = self.create_marker(999,ns, Marker.SPHERE, [self.boid.position.x,self.boid.position.y,0.2], 
            [0.2,0.2,0.2], [0,1,0,1], frame_id)
        self.visualize_array.markers.append(marker)


        for n in self.boid.neighbor_boids:
            marker = self.create_marker(marker_id,ns, Marker.SPHERE, [n.position.x, n.position.y, 0.2], 
            scale, color, frame_id)
            marker_id += 1

            self.visualize_array.markers.append(marker)
        
    
    def visualize_acc(self):
        pass

    def visualize_goal(self):
        pass

    def create_marker(self, marker_id, ns, marker_type, position, scale, color, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id  # Reference frame (change as necessary)
        marker.header.stamp = rospy.Time.now()

        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        # Set marker position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set marker scale
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        # Set marker color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]  # Alpha (transparency)

        return marker

    
        
    

if __name__ == '__main__':
    try:
        id = int(sys.argv[1])
        boid_count = rospy.get_param('boid_count', 5)

        boid_node = BoidNode(id,boid_count)

        rospy.loginfo('Running Boid Node ID: %d' % id)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass