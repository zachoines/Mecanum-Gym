from omni.kit.scripting import BehaviorScript
import omni
from pxr import Usd, UsdLux, UsdGeom, Sdf, Gf, Tf, UsdPhysics
from omni.isaac.core.prims.rigid_prim import RigidPrim, RigidPrimView
from omni.isaac.core.prims.xform_prim import XFormPrim, XFormPrimView
from omni.isaac.core.utils.rotations import quat_to_euler_angles, gf_quat_to_np_array, euler_angles_to_quat
from omni.isaac.core.articulations import ArticulationView
import numpy as np

class MechanumController(BehaviorScript):
    def on_init(self):
        self.base_path = "/World/Mechanum_Robot"
        self.wheel_names = ["TR", "BR", "TL", "BL"]
        self.wheels = []
        print(f"{__class__.__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        print(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        usd_context = omni.usd.get_context()
        stage = usd_context.get_stage()
        # self.mechanum_robot_prim = stage.GetPrimAtPath(self.base_path)
        self.mechanum_robots = ArticulationView(
            prim_paths_expr=self.base_path,
            name="mechanum_robot_view"
        )
        dof_pos = self.mechanum_robots.get_joint_positions()
        dof_vel = self.mechanum_robots.get_joint_velocities()
        self._num_envs = self.mechanum_robots.count
        self._dof = self.mechanum_robots.num_dof
        self.wheel = MechanumWheel(stage, self.base_path + "/" + self.wheel_names[2])

    def on_pause(self):
        pass
        # print(f"{__class__.__name__}.on_pause()->{self.prim_path}")
 
    def on_stop(self):
        pass
        # print(f"{__class__.__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        self.wheel.update()


class MechanumWheel():
    def __init__(
        self,
        stage: object,
        base_path: str
    ) -> None:
        self.base_path = base_path
        self.roller_path = self.base_path + "/Rollers"
        self.num_rollers = 10
        self.rollers = {}

        # Retrieve properities for wheel
        self.revolute_joint = UsdPhysics.RevoluteJoint.Get(
            stage,
            self.base_path + "/RevoluteJoint"
        )

        self.articulation = ArticulationView(
            prim_paths_expr=self.base_path,
            name="wheel_view"
        )

        # Retrieve Properties for rollers
        base_roller_path = self.roller_path + "/Roller_Covers/Roller_Cover_"
        for roller in range(self.num_rollers):
            roller_path = base_roller_path + str(roller)
            self.rollers[roller] = {}
            self.rollers[roller]["prim_path"] = roller_path
            self.rollers[roller]["rp_view"] = RigidPrimView(roller_path)
            self.rollers[roller]["joint"] = UsdPhysics.RevoluteJoint.Get(
                stage,
                roller_path + "/RevoluteJoint"
            )
            # self.roller[roller]['mass'] =  mass_api = UsdPhysics.MassAPI.Get(stage, prim.GetPath())
            # if mass_api is None:
            #     mass_api = UsdPhysics.MassAPI.Apply(prim)
            # mass_attr = mass_api.GetMassAttr()

        test = "test"
    
    def update(self):
        pass
