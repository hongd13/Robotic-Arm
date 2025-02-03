import numpy as np
import cv2
import random

from util import display_image

from detector import Detector
from navigator import Navigator


class Mission_Control():
    
    def __init__(self, robot):
        self.origin = [64, 32] # Centroid of the camera's pov
        self.dest = [-0.78, 0, 0.75] # location of destination (crate)
        self.cubes = 5 # number of cubes
    
        self.tick = True # tick flag
        
        self.robot = robot
        
        self.det = Detector()
        self.nav = Navigator(self.robot)

        
    def run(self, img):
        display_image(img, 'camera view', wait=False)
        #self.cubes -= 1
        
        #self.robot.move_in_axis(0.02, 0, 0)
        #self.robot.go_to_pos(self.dest)
        
        #self.robot.rotate_camera(2)
        
        cube_centroid, cube_pos, cluster = self.det.get_closest_cube(img, self.origin)
        if self.robot.grabbing:
            print("I have a cube!")
            if self.det.holding_cube(img, self.origin):
                print("Heading to drop zone!")
                if self.nav.drop_zone(self.dest):
                    print("Cube in the crate!")
                    self.robot.release()
                    self.cubes -= 1
                    self.robot.move_to_joint_pos(self.robot.home_pos)
            else:
                print("Oops I dropped it...")
                self.robot.release()
                
        elif self.det.cube_aligned(cube_pos, cube_centroid, self.origin):
            print("Lets pick up this cube!")
            self.nav.pick_up()
        
        elif self.det.can_pick_up(cube_centroid, self.origin):
            print("Lets align with this cube!")
            self.nav.align()
            
        elif cluster:
            print("There's a cluster, lets break it up!")
            self.nav.break_cluster(cube_centroid, self.origin)
            
        elif cube_centroid:
            print("Lets get to that cube!")
            #print(cube_pos)
            random.choice([
                self.nav.step_towards(cube_centroid, self.origin, 0.001),
                self.nav.step_towards(cube_centroid, self.origin, 0.002),
                self.nav.step_towards(cube_centroid, self.origin, 0.0004)])

        else:
            print("Lets keep searching!")
            random.choice([
                self.robot.go_to_pos([2, 2, 2]),
                self.robot.go_to_pos([-2, 2, 2]),
                self.robot.go_to_pos([-2, -2, 2])
            ])

        #print(self.cubes)
        if self.cubes <= 0:
            self.tick = False
        return self.tick
