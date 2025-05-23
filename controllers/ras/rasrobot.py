from controller import Supervisor
import numpy as np

TIME_STEP = 32


class RASRobot:

    def __init__(self):
        self.__sup = Supervisor()
        self.__sup.getDevice("camera").enable(TIME_STEP)
        self.__total_cubes = 5
        
        # set motor velocity, initialise sensors
        self.motors = [
            self.__sup.getDevice("shoulder_pan_joint"),
            self.__sup.getDevice("shoulder_lift_joint"),
            self.__sup.getDevice("elbow_joint"),
            self.__sup.getDevice("wrist_1_joint"),
            self.__sup.getDevice("wrist_2_joint"),
            self.__sup.getDevice("wrist_3_joint"),
        ]
        for m in self.motors:
            m.getPositionSensor().enable(TIME_STEP)
            m.setVelocity(0.8)
        
        # initialise fingers
        self.__fingers = [
            self.__sup.getDevice('ROBOTIQ 2F-85 Gripper::left finger joint')
            # right finger mimics the left finger, so we only need to control one of them
        ]
        for finger in self.__fingers:
            finger.setVelocity(0.8)
            
        # shuffle the cubes
        self.__reset_scene()
         
    def __reset_scene(self):
        rng = np.random.default_rng()
        
        for i in range(self.__total_cubes):
            box = self.__sup.getFromDef(f'BOX{i+1}')
            rotation_field = box.getField('rotation')
            quaternion = rng.standard_normal(4)
            quaternion = quaternion / np.linalg.norm(list(quaternion))
            rotation_field.setSFRotation(quaternion.tolist())
       
    def close_gripper(self, timeout=1.2):
        """
        blocking behaviour that will close the gripper
        """
        for finger in self.__fingers:
            finger.setTorque(finger.getAvailableTorque()/1)
            
        for step in range(int(timeout * 1000) // TIME_STEP):
            self.step()
         
    def open_gripper(self, timeout=1.2):
        """
        blocking behaviour that will open the gripper
        """
        for finger in self.__fingers:
            finger.setPosition(0)
            
        for step in range(int(timeout * 1000) // TIME_STEP):
            self.step()
          
    def step(self):
        """
        step function of the simulation
        """
        self.__sup.step()
    
    def get_camera_image(self):
        """
        This method returns a NumPy array representing the latest image captured by the camera.
        It will have 64 rows, 128 columns and 4 channels (red, green, blue, alpha).
        """
        return np.frombuffer(self.__sup.getDevice("camera").getImage(), np.uint8).reshape((64,128,4))
    
