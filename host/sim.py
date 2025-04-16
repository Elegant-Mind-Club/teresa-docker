# MuJoCo Simulation
import socket
import json
import threading
import os
import mujoco
from models.robot import RobotArm, RobotJoint
import zmq


class Listener:
    def __init__(self, callback, ip="0.0.0.0", port=12346):
        self.ip = ip
        self.port = port
        self.callback = callback
        context = zmq.Context()
        try:
            self.s = context.socket(zmq.PULL)
            self.s.bind(f"tcp://*:{self.port}")
        except:
            print("Unable to create socket")
            exit(-1)

    def start(self):
        self.listen_thread = threading.Thread(target=self._listen_loop)
        self.listen_thread.start()
        print("Setup complete")

    def _listen_loop(self):
        while True:
            try:
                message = self.s.recv_string()
                if message:
                    # print(f"Got message: {message}")
                    self.callback(message)
                # self.callback(message)
            except Exception as e:
                print(f"Something went wrong: {e}")


# TODO: add simulated cameras
class SimVis:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(
            os.path.join(os.path.dirname(__file__), "barebones_urdf/mjmodel.xml")
        )
        self.data = mujoco.MjData(self.model)
        self.robot_arm = RobotArm(self.data, self.model)
        # Run thread for listening
        self.listener = Listener(self.set_absolute_position)
        self.listener.start()
        # Run thread for visualization
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer_thread = threading.Thread(target=self.run_viewer)
        self.viewer_thread.start()

    def set_joint(self, joint_name: str, joint_type: str, joint_angle: float):
        if joint_type == "Servo":
            index = self.model.joint(joint_name).qposadr[0]
            self.data.ctrl[index] = joint_angle
            self.data.qpos[index] = joint_angle
            self.data.qvel[index] = 0
            self.data.qacc[index] = 0
        else:
            # Gripper
            subjoint_names = [
                "gripper_joint_a",
                "gripper_joint_b",
                "gripper_claw_a",
                "gripper_claw_b",
            ]
            for joint in subjoint_names:
                self.data.ctrl[index] = joint_angle
                self.data.qpos[index] = joint_angle
                self.data.qvel[index] = 0
                self.data.qacc[index] = 0

    def set_absolute_position(self, joint):
        # joints: array of length 5 in radians specifying joint angles
        self.set_joint("base_joint", "Servo", joint)
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def run_viewer(self):
        self.viewer.cam.distance = 3
        self.viewer.sync()
        while self.viewer.is_running():
            self.viewer.sync()


if __name__ == "__main__":
    # l = Listener(lambda data: print(data))
    # l.start()
    v = SimVis()
