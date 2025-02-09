import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
import transformations

from util import display_image
from rasrobot import RASRobot, TIME_STEP

from util import gimp_hsv_to_cv2

from mission_control import Mission_Control


class UR5e(RASRobot):
    def __init__(self):
        """
        This is your main robot class. It inherits from RASRobot for access
        to devices such as motors, position sensors, and camera.
        """
        super().__init__()
        
        # load the kinematic chain based on the robot's URDF file
        end_link = 'wrist_3_link'  # link used for forward and inverse kinematics
        URDF_FN = '../../resources/ur5e_2f85_camera.urdf'
        self.chain = kp.build_serial_chain_from_urdf(open(URDF_FN), end_link)
        
        # print chain on console
        print('kinematic chain:')
        print(self.chain)
        print(f'The end link of the chain is <{end_link}>.')
        print('All computations of forward and inverse kinematics apply to this link.')
        
        # gripper closed
        self.grabbing = False
        
        
    @property
    def home_pos(self):
        """ 
        this is the home configuration of the robot 
        """
        return [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]

    def joint_pos(self):
        """
        :return: ndarray, the current joint position of the robot
        """
        joint_pos = np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        return joint_pos
        
    def move_to_joint_pos(self, target_joint_pos, timeout=5, velocity=0.8):
        """
        blocking behaviour, moves the robot to the desired joint position.
        :param target_joint_pos: list/ndarray with joint configuration
        :param timeout: float, timeout in seconds after which this function returns latest
        :param velocity: float, target joint velocity in radians/second
        :return: bool, True if robot reaches the target position
                  else will return False after timeout (in seconds)
        """
        if len(target_joint_pos) != len(self.motors):
            raise ValueError('target joint configuration has unexpected length')
            
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
            if velocity is not None:
                motor.setVelocity(velocity)
            
        # step through simulation until timeout or position reache
        for step in range(int(timeout * 1000) // TIME_STEP):
            self.step()

            # check if the robot is close enough to the target position
            if all(abs(target_joint_pos - self.joint_pos()) < 0.001):
                return True
                
        print('Timeout. Robot has not reached the desired target position.')
        return False
        
    def forward_kinematics(self, joint_pos=None):
        """
        computes the pose of the chain's end link for given joint position.
        :param joint_pos: joint position for which to compute the end-effector pose
                          if None given, will use the robot's current joint position
        :return: kinpy.Transform object with pos and rot
        """
        if joint_pos is None:
            joint_pos = self.joint_pos()
            
        ee_pose = self.chain.forward_kinematics(joint_pos)
        return ee_pose
        
    def inverse_kinematics(self, target_pose):
        """
        Computes a joint configuration to reach the given target pose.
        Note that the resulting joint position might not actually reach the target
        if the target is e.g. too far away.
        :param target_pose: kinpy.Transform, pose of the end link of the chain
        :return: list/ndarray, joint position
        """
        ik_result = self.chain.inverse_kinematics(target_pose, self.joint_pos())
        return ik_result
   
   
   
    def rotate_camera(self, deg):
        """
        Rotates the camera horizontally
        
        params:
        deg - rotation degrees
        
        returns:
        none
        """
        tf = robot.forward_kinematics()
        rotation_axis = [0, 0, 1]
        rotation_angle = np.deg2rad(deg)
        my_rotation = transformations.quaternion_about_axis(rotation_angle, rotation_axis)
        new_rot = transformations.quaternion_multiply(my_rotation, tf.rot)
        tf.rot = new_rot
        
        joint_pos = robot.inverse_kinematics(tf)
        robot.move_to_joint_pos(joint_pos)
        
    def set_camera_orientation(self, axis, deg):
        """
        Rotates the camera per the specified orientation and degree
        
        params:
        axis - 0, 1, 2 axis
        deg - rotation degrees
        
        returns:
        none
        """
        tf = robot.forward_kinematics()
        rotation_angle = np.deg2rad(deg)
        my_rotation = transformations.quaternion_about_axis(rotation_angle, axis)
        new_rot = transformations.quaternion_multiply(my_rotation, tf.rot)
        tf.rot = new_rot
        
        joint_pos = robot.inverse_kinematics(tf)
        robot.move_to_joint_pos(joint_pos)

    def get_camera_rad(self):
        """
        Convert joint pos to camera rad
        
        returns:
        rad - camera angle in rad
        """
        deg = np.ceil(-robot.joint_pos()[5] * (90 / 1.57))
        rad = np.deg2rad(deg)
        return rad
        
    def go_to_pos(self, pos):
        """
        End effector go to given xyz position
        
        params:
        pos - xyz positional array
        
        returns:
        none
        """
        tf = robot.forward_kinematics()
        tf.pos = np.array(pos)
        joint_pos = robot.inverse_kinematics(tf)
        robot.move_to_joint_pos(joint_pos)
        
    def move_in_axis(self, x, y, z):
        """
        Change the current position by the specified amount
        
        params:
        x - amount in x axis
        y - amount in y axis
        z - amount in z axis
        
        returns:
        none
        """
        tf = robot.forward_kinematics()
        tf.pos[0] = tf.pos[0] + x
        tf.pos[1] = tf.pos[1] + y
        tf.pos[2] = tf.pos[2] + z
        robot.go_to_pos(tf.pos)
        
    def move_towards(self, pos, pace):
        """
        Given a target position, move toward it with the specified pace
        
        params:
        pos - xyz array
        pace - rate of change
        
        returns:
        bool - True - reached
                False - not reached
        """
        tf = robot.forward_kinematics()
        cur_pos = tf.pos
        
        if abs(cur_pos[2] - pos[2]) < (pace)*10 and abs(cur_pos[0] - pos[0]) < pace*10 and abs(cur_pos[1] - pos[1]) < pace*10:
            return True
            
        if abs(cur_pos[2] - pos[2]) > (pace/10)*2.5:
            if cur_pos[2] > pos[2]:
                cur_pos[2] -= pace/10
            elif cur_pos[2] < pos[2]:
                cur_pos[2] += pace/10
        if abs(cur_pos[0] - pos[0]) > pace*2.5:
            if cur_pos[0] > pos[0]:
                cur_pos[0] -= pace
            elif cur_pos[0] < pos[0]:
                cur_pos[0] += pace
        if abs(cur_pos[1] - pos[1]) > pace*2.5:
            if cur_pos[1] > pos[1]:
                cur_pos[1] -= pace
            elif cur_pos[1] < pos[1]:
                cur_pos[1] += pace
        robot.go_to_pos(cur_pos)
        
        return False
            
    def grab(self):
        """
        Close gribber and set flag
        """
        robot.close_gripper()
        self.grabbing = True
        
    def release(self):
        """
        Open gripper and unset flag
        """
        robot.open_gripper()
        self.grabbing = False
        
        
if __name__ == '__main__':
    # initialise robot and move to home position
    robot = UR5e()
    
    print("Initialising")
    robot.move_to_joint_pos(robot.home_pos)
    
    mc = Mission_Control(robot)
    img = robot.get_camera_image()
    while mc.run(img):
        img = robot.get_camera_image()

    print("Terminating")
    robot.move_to_joint_pos(robot.home_pos)

