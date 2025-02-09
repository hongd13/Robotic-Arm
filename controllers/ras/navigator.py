import numpy as np

class Navigator():

    def  __init__(self, robot):
        self.robot = robot

    def drop_zone(self, dest):
        """
        Go to drop zone
        
        params:
        dest - crate location
        
        returns:
        reached - false = not reached
                   true = reached
        """
        """
        reached = False
        while reached == False:
            reached = self.robot.move_towards([-0.78, 0, 0.75], 0.01)
            """
        reached = self.robot.move_towards([-0.78, 0, 0.75], 0.01)
        return reached

    def pick_up(self):
        """
        Lower the gripper to pick up if the cube size is under 300
        
        params:
        none
        
        returns;
        none
        """
        for i in range(0, 32, 1):
            # go down to grab
            self.robot.move_in_axis(0, 0, -0.01)
        self.robot.grab()

        # straight down and up
        #"""
        for i in range(0, 32, 1):
            # come back up
            self.robot.move_in_axis(0, 0, 0.01)
        #"""
        
        # natural cupping - set origin offset to near the cube's edge
        """
        for i in range(0, 90, 1):
            rad = self.robot.get_camera_rad()
            self.robot.set_camera_orientation([np.cos(rad), np.sin(rad), 0], 1)
        """
        
        # natural cupping to palm rest
        """
        for i in range(0, 90, 1):
            rad = self.robot.get_camera_rad()
            self.robot.set_camera_orientation([np.cos(rad), np.sin(rad), 0], 1)
        for i in range(0, 90, 1):
            self.robot.set_camera_orientation([0, 1, 0], 1)
        """
        
        # tilt then scoop
        """
        for i in range(0, 30, 1):
            rad = self.robot.get_camera_rad()
            self.robot.set_camera_orientation([-np.sin(rad), np.cos(rad), 0], 1)
        rad = self.robot.get_camera_rad()
        self.robot.set_camera_orientation([-np.sin(rad), np.cos(rad), 0], 60)
        """
        
    def align(self):
        """
        Rotate the camera to find the opitimal pick-up angle
        """
        self.robot.rotate_camera(0.5)
        
    def break_cluster(self, target, origin):
        """
        Taken a target position, lower down, and penetrate target cluster
        
        params:
        target - cluster centroid
        origin - camera cnetroid
        
        return:
        none
        """
        self.robot.move_in_axis(0, 0, -0.55)
        self.step_towards(target, origin, 5)
        self.robot.move_to_joint_pos(self.robot.home_pos)
        return
        
    def step_towards(self, target, origin, speed):
        """
        Receive the target position, origin and speed, 
        determine the direction and move towards the target 
        with the specified speed per tick 
        
        params:
        target - x, y
        origin - camera centroid
        speed - delta per tick
        
        returns:
        none
        """
        xt, yt = target
        xo, yo = origin
        yo += 13 # origin offset, 13 = centre, 10 = edge
        xo -= 1
        
        rad = self.robot.get_camera_rad()
        # move in x axis first
        if abs(xt - xo) > 0.5:
            if xt > xo:
                self.robot.move_in_axis(np.cos(rad)*speed, 
                                    np.sin(rad)*speed, 0)
            else:
                self.robot.move_in_axis(np.cos(rad)*-speed, 
                                    np.sin(rad)*-speed, 0)
            return
        # then move in y
        elif abs(yt - yo) > 0.5:
            if yt > yo:
                self.robot.move_in_axis(-np.sin(rad)*-speed, 
                                            np.cos(rad)*-speed, 0)
            else:
                self.robot.move_in_axis(-np.sin(rad)*speed, 
                                            np.cos(rad)*speed, 0)
            return
        else:
            return True