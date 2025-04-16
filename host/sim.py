# MuJoCo Simulation
import threading
import os
import mujoco
import zmq
import time


class Listener:
    def __init__(self, callback, ip="0.0.0.0", port=12346):
        self.ip = ip
        self.port = port
        self.callback = callback
        context = zmq.Context()
        try:
            self.s = context.socket(zmq.PULL)
            self.s.setsockopt(zmq.CONFLATE, 1)
            self.s.bind(f"tcp://*:{self.port}")
        except Exception as e:
            print(f"Unable to create socket: {e}")
            exit(-1)

    def start(self):
        self.listen_thread = threading.Thread(target=self._listen_loop)
        self.listen_thread.start()
        print("Listening to the container...")

    def _listen_loop(self):
        while True:
            try:
                message = self.s.recv_string()
                if message:
                    message_array = list(map(float, message.split(",")))
                    self.callback(message_array)
            except Exception as e:
                print(f"Something went wrong: {e}")


# TODO: add simulated cameras
class SimVis:
    def __init__(self):
        self.SIM_TARGET_FPS = 30  # FPS
        self.visualization_refresh_rate = 1 / self.SIM_TARGET_FPS  # Hz
        self.model = mujoco.MjModel.from_xml_path(
            os.path.join(os.path.dirname(__file__), "barebones_urdf/mjmodel.xml")
        )
        self.data = mujoco.MjData(self.model)
        # Run thread for visualization
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer_thread = threading.Thread(target=self.run_viewer)
        self.viewer_thread.start()
        # Run thread for listening
        self.listener = Listener(self.set_absolute_position)
        self.listener.start()

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
                index = self.model.joint(joint).qposadr[0]
                self.data.ctrl[index] = joint_angle
                self.data.qpos[index] = joint_angle
                self.data.qvel[index] = 0
                self.data.qacc[index] = 0

    def set_absolute_position(self, joints: list[float]):
        # joints: array of length 5 in radians specifying joint angles
        for joint_name, joint_value in zip(
            ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"], joints[0:-1]
        ):
            self.set_joint(joint_name, "Servo", joint_value)
        self.set_joint("gripper_joint", "Gripper", joints[-1])
        mujoco.mj_forward(self.model, self.data)

    def run_viewer(self):
        # Initialize the viewer
        self.viewer.cam.distance = 3

        self.viewer.sync()
        self.last_sync_time = time.time()
        # Main viewer loop (refresh based on target FPS)
        while self.viewer.is_running():
            curr_time = time.time()
            if curr_time - self.last_sync_time > self.visualization_refresh_rate:
                self.viewer.sync()
                self.last_sync_time = curr_time


if __name__ == "__main__":
    v = SimVis()
