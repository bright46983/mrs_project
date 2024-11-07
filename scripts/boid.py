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
        self.max_vel = 1.0
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
            # for n_boid in neighbor_boids:
                
            #     sep_acc.x = ...
            #     sep_acc.y = ...

            return self.limit_acc(sep_acc)

        else:
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
            # for n_boid in neighbor_boids:
                
            #     allign_acc.x = ...
            #     allign_acc.y = ...
            return self.limit_acc(allign_acc)
        else:
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

        return nav_acc
    
    def combine_acc(self, nav_acc,coh_acc, allign_acc, sep_acc, obs_acc):
        return nav_acc

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

