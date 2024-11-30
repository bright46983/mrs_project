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
            avg_position = Point()
            for n_boid in neighbor_boids:
                avg_position.x += n_boid.position.x
                avg_position.y += n_boid.position.y

            avg_position.x /= len(neighbor_boids)
            avg_position.y /= len(neighbor_boids)
    
            coh_acc.x = avg_position.x - self.position.x
            coh_acc.y = avg_position.y - self.position.y

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

    #######################
    #### Navigation Methods
    #######################
    def seek(self):
        """
        Computes the required acceleration for the boid to seek its goal.
        """

        if self.goal is None:
            return Point()

        distance = np.array([self.position.x, self.position.y]) - np.array([self.goal.x, self.goal.y])
        squared_distance = np.linalg.norm(distance)**2
        squared_distance = max(squared_distance, 1e-5)  # Avoid division by zero

        seek_acc = distance / squared_distance
        return self.limit_acc(seek_acc)

    def arrival(self):
        """"
            Computes the required acceleration for the boid to arrive at its goal.
        """

        if self.goal is None:
            return Point()

        target_offset = np.array([self.goal.x, self.goal.y]) - np.array([self.position.x, self.position.y])
        distance = np.linalg.norm(target_offset)

        if distance < getattr(self.goal, "z", 1.0):   #  (z) represents the tolerance or threshold distance (reached the goal).
            return Point()

        ramped_speed = self.max_vel * (distance / getattr(self.goal, "z", 1.0))
        clipped_speed = min(ramped_speed, self.max_vel)
        desired_velocity = (clipped_speed / distance) * target_offset

        arrival_acc = (desired_velocity - np.array([self.velocity.x, self.velocity.y]))

        return self.limit_acc(arrival_acc)


    ########################
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

