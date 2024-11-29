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
        self.nav_gain = 1.0  # Navigation gain, controls the strength of the navigation behavior
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

        for n_boid in self.neighbor_boids:
            distance = np.linalg.norm(
                np.array([self.position.x, self.position.y]) - np.array([n_boid.position.x, n_boid.position.y])
            )
            if distance > 0:
                # Calculate repulsion vector inversely proportional to distance
                sep_acc.x += (self.position.x - n_boid.position.x) / distance**2
                sep_acc.y += (self.position.y - n_boid.position.y) / distance**2

        return self.limit_acc(sep_acc)
        
    def cohesion_acc(self):
        """
        Calculate the seperation acceleration of the boid from its neighbors.
        - neighbors list contain boid in perception range [[boid#2], [boid#5],...]
        - Returns a Point() representing the seperation acceleration.
        """

        neighbor_boids = self.neighbor_boids
        coh_acc = Point()

        if self.neighbor_boids:
            center_x = sum(n_boid.position.x for n_boid in self.neighbor_boids) / len(self.neighbor_boids)
            center_y = sum(n_boid.position.y for n_boid in self.neighbor_boids) / len(self.neighbor_boids)

            coh_acc.x = center_x - self.position.x
            coh_acc.y = center_y - self.position.y

        return self.limit_acc(coh_acc)
        
    def allignment_acc(self):
        """
        Calculate the seperation acceleration of the boid from its neighbors.
        - neighbors list contain boid in perception range [[boid#2], [boid#5],...]
        - Returns a Point() representing the seperation acceleration.
        """

        neighbor_boids = self.neighbor_boids
        allign_acc = Point()

        if self.neighbor_boids:
            avg_vel_x = sum(n_boid.velocity.x for n_boid in self.neighbor_boids) / len(self.neighbor_boids)
            avg_vel_y = sum(n_boid.velocity.y for n_boid in self.neighbor_boids) / len(self.neighbor_boids)

            allign_acc.x = avg_vel_x - self.velocity.x
            allign_acc.y = avg_vel_y - self.velocity.y

        return self.limit_acc(allign_acc)
        
    # def obstacle_acc(self):
    #     """
    #     Calculate the obstacle avoidance acceleration using grid map.
    #     - map is Occupancy grid map
    #     - Returns a Point() representing the seperation acceleration.
    #     """

    #     map = self.map
    #     obs_acc = Point()

    #     need_to_avoid = False
    #     if need_to_avoid :
    #         # TODO:
    #         return self.limit_acc(obs_acc)
    #     else:
    #         return self.limit_acc(obs_acc)

    def obstacle_acc(self):
        """
        Calculate the obstacle avoidance acceleration using grid map.
        - map is an Occupancy grid map.
        - Returns a Point() representing the separation acceleration.
        """
        map = self.map
        obs_acc = Point()

        # Get the robot's position in the grid
        i = int((self.position.x - map.info.origin.position.x) / map.info.resolution)
        j = int((self.position.y - map.info.origin.position.y) / map.info.resolution)

        # Iterate over grid cells within a certain range of the robot's position
        range_x = range(i - 20, i + 21)
        range_y = range(j - 20, j + 21)

        for x in range_x:
            for y in range_y:
                if 0 <= x < map.info.width and 0 <= y < map.info.height:
                    index = y * map.info.width + x
                    if map.data[index] > 50:  # Consider as an obstacle if occupancy is above 50
                        obs_vector_x = self.position.x - (x * map.info.resolution + map.info.origin.position.x)
                        obs_vector_y = self.position.y - (y * map.info.resolution + map.info.origin.position.y)
                        distance = np.hypot(obs_vector_x, obs_vector_y)
                        if distance > 0:
                            obs_acc.x += obs_vector_x / distance**2
                            obs_acc.y += obs_vector_y / distance**2

        return self.limit_acc(obs_acc)



    def navigation_acc(self):
        nav_acc = Point()
        if self.goal:
            nav_acc.x = (self.goal.x - self.position.x) * self.nav_gain
            nav_acc.y = (self.goal.y - self.position.y) * self.nav_gain

        return nav_acc
    
    def combine_acc(self, nav_acc,coh_acc, allign_acc, sep_acc, obs_acc):
        combined_acc = Point()
        combined_acc.x = (nav_acc.x + coh_acc.x + allign_acc.x + sep_acc.x + obs_acc.x) / 5
        combined_acc.y = (nav_acc.y + coh_acc.y + allign_acc.y + sep_acc.y + obs_acc.y) / 5

        return self.limit_acc(combined_acc)

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

