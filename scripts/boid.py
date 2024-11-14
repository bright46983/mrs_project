import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from random import uniform
import numpy as np

class Boid:
    def __init__(self):
        # Initialize Boid's state
        self.position = Point()
        self.velocity = Point()
        self.heading = 0.0
        self.goal = None
        self.map = None
        
        # Random initial position
        self.position.x = 0
        self.position.y = 0
        
        # Random initial velocity
        self.velocity.x = 0
        self.velocity.y = 0

        #tuning params
        self.max_acc = 3.0
        self.max_vel = 0.5
        self.nav_gain = 3.0  # Navigation gain, controls the strength of the navigation behavior
        self.neighbor_range = 1
        
        self.other_boids = []
        self.neighbor_boids = []

    # def update_velocity(self, vel ):
    #     # Update velocity 
    #     self.velocity = self.limit_vel(vel)
    
    ##################################################
    #### Perception 
    ##################################################

    def update_neigbors(self,other_boids):
    
        self.neighbor_boids = []  # Reset neighbor list
        for o_boid in other_boids:
            if o_boid is not None:
                dis = np.linalg.norm(np.array([self.position.x, self.position.y])-np.array([o_boid.position.x, o_boid.position.y]))
                if dis < self.neighbor_range :
                    self.neighbor_boids.append(o_boid)

        return self.neighbor_boids
        

    
    
    ##################################################
    #### Acceleration calculation
    ##################################################

    def seperation_acc(self):
        """
        Calculate the seperation acceleration of the boid from its neighbors.
        - neighbors list contain boid in perception range [[boid#2], [boid#5],...]
        - Returns a Point() representing the seperation acceleration. (or vel not sure)
        """

        neighbor_boids = self.neighbor_boids
        sep_acc = Point()

        if neighbor_boids != []:
            # Iterate through each neighbor boid
            for n_boid in neighbor_boids:
                
                # Calculate the distance between the current boid and its neighbor
                dis = np.linalg.norm(np.array([self.position.x, self.position.y])-np.array([n_boid.position.x, n_boid.position.y]))

                # Avoid division by zero
                if dis > 0:  # Not sure if it should be exactly 0 or maybe very small value
                    # Repulsion force: inverse proportionality to the distance
                    # This creates a vector pointing away from the neighboring boid.
                    sep_acc.x += (self.position.x - n_boid.position.x)/dis
                    sep_acc.y += (self.position.y - n_boid.position.y)/dis

            # Normalize the acceleration vector
            magnitude = np.linalg.norm(np.array([sep_acc.x, sep_acc.y]))
            if magnitude > 0:
                sep_acc.x /= magnitude
                sep_acc.y /= magnitude

            return self.limit_acc(sep_acc)

        # Return zero acceleration if no neighbors are present
        return self.limit_acc(sep_acc)
        
    def cohesion_acc(self):
        """
        Calculate the seperation acceleration of the boid from its neighbors.
        - neighbors list contain boid in perception range [[boid#2], [boid#5],...]
        - Returns a Point() representing the seperation acceleration.
        """

        neighbor_boids = self.neighbor_boids
        coh_acc = Point()

        if neighbor_boids != []:
            # for n_boid in neighbor_boids:
                
            #     coh_acc.x = ...
            #     coh_acc.y = ...

            return self.limit_acc(coh_acc)

        else:
            return self.limit_acc(coh_acc)
        
    def allignment_acc(self):
        """
        Calculate the seperation acceleration of the boid from its neighbors.
        - neighbors list contain boid in perception range [[boid#2], [boid#5],...]
        - Returns a Point() representing the seperation acceleration.
        """

        neighbor_boids = self.neighbor_boids
        allign_acc = Point()

        if neighbor_boids != []:
            # container to store average velocity of neighboring boids
            xvel_avg = 0
            yvel_avg = 0

            # loop through all neighboring boids
            for n_boid in neighbor_boids:
                # sum the velocity of neighboring boids
                xvel_avg += n_boid.velocity.x
                yvel_avg += n_boid.velocity.y

            # take the average velocity of neighboring boids as the "desired velocity"
            xvel_avg /= len(neighbor_boids)
            yvel_avg /= len(neighbor_boids)

            # compute necessary acceleration                
            allign_acc.x = xvel_avg - self.velocity.x 
            allign_acc.y = yvel_avg - self.velocity.y 
        else:
            allign_acc.x = 0
            allign_acc.y = 0
            
        return self.limit_acc(allign_acc)
        
    def obstacle_acc(self):
        """
        Calculate the obstacle avoidance acceleration using grid map.
        - map is Occupancy grid map
        - Returns a Point() representing the seperation acceleration.
        """

        map = self.map
        obs_acc = Point()

        need_to_avoid = False
        if need_to_avoid :
            # TODO:
            return self.limit_acc(obs_acc)
        else:
            return self.limit_acc(obs_acc)


    def navigation_acc(self):
        nav_acc = Point()
        if self.goal:
            nav_acc.x = (self.goal.x - self.position.x) * self.nav_gain
            nav_acc.y = (self.goal.y - self.position.y) * self.nav_gain

        return self.limit_acc(nav_acc)
    
    def combine_acc(self, nav_acc,sep_acc,coh_acc,allign_acc,obs_acc):
        combined_acc = Point()
        combined_acc.x = nav_acc.x  +allign_acc.x + 3*sep_acc.x
        combined_acc.y = nav_acc.y  +allign_acc.y + 3*sep_acc.y

        combined_acc.x = nav_acc.x  
        combined_acc.y = nav_acc.y  

        # rospy.loginfo("nav,coh,allign,sep,obs,com [x]: {},{},{},{},{}".format(nav_acc.x,coh_acc.x, allign_acc.x, sep_acc.x, obs_acc.x,combined_acc.x))
        # rospy.loginfo("nav,coh,allign,sep,obs,com [y]: {},{},{},{},{}".format(nav_acc.y,coh_acc.y, allign_acc.y, sep_acc.y, obs_acc.y,combined_acc.y))

        return combined_acc

    ##################################################
    #### helper functions
    ##################################################
    def cal_velocity(self, acc , dt):
        # Update velocity 
        vel = Point()
        vel.x = self.velocity.x + (acc.x*dt)
        vel.y = self.velocity.y + (acc.y*dt)

        out_vel = self.limit_vel(vel)
        return out_vel
    
    def limit_vel(self,vel):
        # Limit the vel of the boid to avoid it moving too fast
        if abs(vel.x) > self.max_vel:
            vel.x = (vel.x/abs(vel.x)) * self.max_vel
        
        if abs(vel.y) > self.max_vel:
            vel.y = (vel.y/abs(vel.y)) * self.max_vel

        return vel
    
    def limit_acc(self,acc):
        # Limit the acc of the boid to avoid it moving too fast
        if abs(acc.x) > self.max_acc:
            acc.x = (acc.x/abs(acc.x)) * self.max_acc
        
        if abs(acc.y) > self.max_acc:
            acc.y = (acc.y/abs(acc.y)) * self.max_acc

        return acc

