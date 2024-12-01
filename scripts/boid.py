import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from random import uniform
import numpy as np

from utils.OccupancyMap import OccupancyMap
from utils.SteerToAvoid import SteerToAvoid

class Boid:
    def __init__(self):
        # Initialize Boid's state
        self.position   = Point()
        self.velocity   = Point()
        self.heading    = 0.0
        self.goal       = None
        self.perception_field       = OccupancyMap()
        self.perception_radius      = 1.0  
        
        # Random initial position
        self.position.x = 0
        self.position.y = 0
        
        # Random initial velocity
        self.velocity.x = 0
        self.velocity.y = 0

        #tuning params
        self.max_acc    = 2.5
        self.max_vel    = 1.2
        self.nav_gain   = 1.0  # Navigation gain, controls the strength of the navigation behavior
        self.neighbor_range = 1
        
        self.other_boids = []
        self.neighbor_boids = []

        self.obs_acc = SteerToAvoid(0.6, 0.174533, 6.28)

    # def update_velocity(self, vel ):
    #     # Update velocity 
    #     self.velocity = self.limit_vel(vel)
    
    ##################################################
    #### Perception 
    ##################################################

    def update_neigbors(self, other_boids):
    
        self.neighbor_boids = []  # Reset neighbor list
        for o_boid in other_boids:
            if o_boid is not None:
                dis = np.linalg.norm(np.array([self.position.x, self.position.y])-np.array([o_boid.position.x, o_boid.position.y]))
                if dis < self.neighbor_range :
                    self.neighbor_boids.append(o_boid)

        return self.neighbor_boids
    
    def update_perception_field(self, map:OccupancyMap):
        map_crop, map_dim, resolution, origin, there_is_map = map.crop_pos([self.position.x, self.position.y], self.perception_radius)
        self.perception_field.update(map_crop, map_dim, resolution, origin, there_is_map)
        # self.perception_field.show_orin_map() 
        # print(map_crop[:, 20])
        # print("______________")
        # print(map_crop[22, :])
        self.obs_acc.update_map(self.perception_field)
        

    def obstacle_acc(self):
        boid_pose   = [0.0, 0.0]
        boid_vel    = [self.velocity.x, self.velocity.y]
        # boid_vel    = [0.8, 0.0]
        b = self.obs_acc._steer_to_avoid(boid_pose, boid_vel)
        a = 1
        combined_acc = Point()
        combined_acc.x = b[0]
        combined_acc.y = b[1]
        return combined_acc
        
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


    def _arrival(self):
        ''' Computes required acceleration for the boid to arrive at the goal.'''
        desired_vel = Point()
        desired_vel.x = 0.
        desired_vel.y = 0.
        if self.goal:
            boid_pose = [self.position.x, self.position.y] 
            boid_goal = [self.goal.x, self.goal.y, 1.0, 0.05]
            desired_velocity = np.array([0., 0.])
            if boid_goal != None:
                target_offset = np.array(boid_goal[:2]) - np.array(boid_pose[:2])   # goal[x, y], not r or tolerance. 
                distance = np.linalg.norm(target_offset)
                if distance < boid_goal[3]:
                    # print("Distance: ", distance)
                    #TODO: send a flag that boid_goal has been reached and generate a new boid_goal
                    
                    return desired_vel # Set the distance to 0 when it falls below the tolerance. 

                ramped_speed = self.max_vel * (distance / boid_goal[2])
                clipped_speed = np.minimum(ramped_speed, self.max_vel)
                desired_velocity = (clipped_speed / distance) * target_offset
                # Sample the desired velocity from the velocity space using probability
                # desired_velocity = np.array([random.uniform(-self.max_speed, self.max_speed), random.uniform(-self.max_speed, self.max_speed)]) if random.uniform(0, 1) < 0.5 else desired_velocity
            
            desired_vel.x = desired_velocity[0]*0.3
            desired_vel.y = desired_velocity[1]*0.3
        return desired_vel