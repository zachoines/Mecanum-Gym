from omni.kit.scripting import BehaviorScript
from omni.isaac.dynamic_control import _dynamic_control
import omni
import numpy as np
import math

class MecanumRobotController(BehaviorScript):
    def on_init(self):
        pass

    def on_play(self):
        omni.timeline.get_timeline_interface().play()
        self.robot_root_path = "/World/Mechanum_Robot"
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.robot_articulation = self.dc.get_articulation(self.robot_root_path)
        self.joints = {}

    def on_stop(self):
        pass  # Add any cleanup code here

    def on_update(self, current_time, delta_time):
        self.robot_articulation = self.dc.get_articulation(self.robot_root_path)
        if self.robot_articulation != 0:
            if self.joints == {}:
                self.joints = {
                    "TL": self.dc.get_articulation_dof(self.robot_articulation, 0),
                    "BL": self.dc.get_articulation_dof(self.robot_articulation, 1),
                    "TR": self.dc.get_articulation_dof(self.robot_articulation, 2),
                    "BR": self.dc.get_articulation_dof(self.robot_articulation, 3)
                }

            self.set_mecanum_drives()
            # self.dc.wake_up_articulation(self.robot_articulation)

    def list_all_dofs(self):
        for index in range(self.dc.get_articulation_dof_count(self.robot_articulation)):
            dof = self.dc.get_articulation_dof(self.robot_articulation, index)
            joint_path = self.dc.get_dof_path(dof)
            print(joint_path)


    def calculate_wheel_drives(self, angle, speed, angular_velocity):
        # Convert the angle to radians
        angle_rad = math.radians(angle)

        # Define the linear drive matrix
        linear_drive_matrix = np.array([[np.sin(angle_rad + np.pi/4), 0],
                                        [0, np.cos(angle_rad + np.pi/4)],
                                        [np.cos(angle_rad + np.pi/4), 0],
                                        [0, np.sin(angle_rad + np.pi/4)]])

        # Define the speed vector
        speed_vector = np.array([speed, speed])

        # Calculate the drive signals using matrix multiplication
        drive_signals = np.dot(linear_drive_matrix, speed_vector)

        # Add the rotational velocity to the drive signals
        drive_signals += angular_velocity * np.array([1, 1, -1, -1])

        # Extract individual wheel velocities
        TL, BL, TR, BR = drive_signals

        return TR, BR, TL, BL


    def set_mecanum_drives(self):

        # Extract individual wheel velocities
        TR, BR, TL, BL = self.calculate_wheel_drives(angle=0, speed=2, angular_velocity=0)

        self.dc.set_dof_velocity(self.joints["TL"], TL)
        self.dc.set_dof_velocity(self.joints["BL"], BL)
        self.dc.set_dof_velocity(self.joints["TR"], TR * -1.0)
        self.dc.set_dof_velocity(self.joints["BR"], BR * -1.0)