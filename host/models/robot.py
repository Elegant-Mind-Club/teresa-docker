# Describe important stuff about the robot/arms
import time
from typing import List

class Utils:
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)

class Servo:
    def __init__(self, name: str, min_angle: float, max_angle: float):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.speed = 10
        self.acceleration = 150 # todo 
        self.position = 0
        self.target_position = 0
        self.last_update = time.time()

    def get_name(self):
        return self.name
    
    def get_index(self):
        return self.index
    
    def set_target_position(self, radian: float):
        self.target_position = radian

    def get_target_position(self):
        return self.target_position

    def get_position(self):
        return self.position

    def update(self):
        # self.position = self.target_position
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        max_step = self.speed * dt
        distance_to_target = self.target_position - self.position 
        if abs(distance_to_target) <= max_step:
            self.position = self.target_position
        else:
            self.position += max_step if distance_to_target > 0 else -max_step

class Gripper(Servo):
    def __init__(self, names: List[str], min_angle: float, max_angle: float):
        super().__init__("gripper", min_angle, max_angle)
        self.names = names

    


class RobotJoint:
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST = 3
    GRIPPER = 4

class RobotArm:
    def __init__(self, data, model):
        self.data = data
        self.model = model
        self.joints = {
            RobotJoint.BASE: Servo("base_joint", -3.14, 3.14),
            RobotJoint.SHOULDER: Servo("shoulder_joint", -3.14, 3.14),
            RobotJoint.ELBOW: Servo("elbow_joint", -1.57, 1.57),
            RobotJoint.WRIST: Servo("wrist_joint", -1.57, 1.57),
            RobotJoint.GRIPPER: Gripper(["gripper_joint_a", "gripper_joint_b", "gripper_claw_a", "gripper_claw_b"], -1.57, 0)
        }

    def get_joint_index(self, joint_name: str):
        joint_index = self.model.joint(joint_name).qposadr[0]
        if joint_index == -1:
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        return joint_index

    def set_joint(self, joint: RobotJoint, radian: float, velocity: float, acceleration: float):
        if joint not in self.joints:
            raise ValueError(f"Joint '{joint}' not found in model.")
        actuator: Servo = self.joints[joint]
        radian = Utils.clamp(radian, actuator.min_angle, actuator.max_angle)
        actuator.set_target_position(radian)

    def get_joint(self, joint: RobotJoint):
        if joint not in self.joints:
            raise ValueError(f"Joint '{joint}' not found in model.")
        return self.joints[joint]

    def update(self):
        for joint in self.joints.values():
            joint.update()
            if type(joint) == Gripper:
                for subjoint_name in joint.names:
                    self.data.ctrl[self.get_joint_index(subjoint_name)] = joint.get_target_position()
                    self.data.qpos[self.get_joint_index(subjoint_name)] = joint.get_position()
                    self.data.qvel[self.get_joint_index(subjoint_name)] = 0
                    self.data.qacc[self.get_joint_index(subjoint_name)] = 0
            else:
                self.data.ctrl[self.get_joint_index(joint.get_name())] = joint.get_target_position()
                self.data.qpos[self.get_joint_index(joint.get_name())] = joint.get_position()
                self.data.qvel[self.get_joint_index(joint.get_name())] = 0
                self.data.qacc[self.get_joint_index(joint.get_name())] = 0

    def get_model_position(self, joint_name: str):
        return self.data.qpos[self.get_joint_index(joint_name)]