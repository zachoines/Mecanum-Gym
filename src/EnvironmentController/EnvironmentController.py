from omni.kit.scripting import BehaviorScript
from omni.isaac.dynamic_control import _dynamic_control
from omni.physx import get_physx_scene_query_interface
from omni.physx.scripts.physicsUtils import *
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema
from typing import List
import random
import carb

class EnvironmentController(BehaviorScript):

    def on_init(self):
        self.robot_root_path = "/World/Mechanum_Robot"
        self.goal_path = "/World/Goal"
        self.collision_check_frequency = 1
        self.update_counter = 0
        self.triggerBox = None

    def on_play(self):
        omni.timeline.get_timeline_interface().play()
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.init_goal_trigger()

    def on_update(self, current_time, delta_time):
        if self.check_goal_trigger():
            self.reset_goal_random()

    def init_goal_trigger(self, pos: Gf.Vec3f = Gf.Vec3f(-50.0, 1.0, 6)):
        """"""
        defaultPrimPath = str(self._stage.GetDefaultPrim().GetPath())
        goalTriggerPath = defaultPrimPath + "/goalTrigger"

        # Check if trigger exists
        self.goalUsdPrim = self._stage.GetPrimAtPath(self.goal_path)
        if not self.triggerBox:
            self.triggerBox = UsdGeom.Cube.Define(self._stage, goalTriggerPath)
            self.triggerBox.CreateSizeAttr(10)
            self.triggerBox.AddTranslateOp().Set(pos)
            self.triggerBox.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
            self.triggerBox.CreatePurposeAttr(UsdGeom.Tokens.guide)
        else:
          self.triggerBox.ClearXformOpOrder()
          self.triggerBox.AddTranslateOp().Set(pos)  
        self.goalTriggerUsdPrim = self._stage.GetPrimAtPath(goalTriggerPath)
        UsdPhysics.CollisionAPI.Apply(self.goalTriggerUsdPrim)
        PhysxSchema.PhysxTriggerAPI.Apply(self.goalTriggerUsdPrim)
        self.triggerStateAPI = PhysxSchema.PhysxTriggerStateAPI.Apply(self.goalTriggerUsdPrim)
        self.set_pose(goalTriggerPath, self.goal_path)

    def set_pose(self, path_a, path_b):
        """"""
        stage = self._stage
        
        # Get the prims from the paths
        prim_a = stage.GetPrimAtPath(path_a)
        prim_b = stage.GetPrimAtPath(path_b)

        # Ensure both prims exist
        if not prim_a or not prim_b:
            print(f"Prim(s) not found: {path_a if not prim_a else ''} {path_b if not prim_b else ''}")
            return

        # Get the transform of the first prim
        pose = omni.usd.utils.get_world_transform_matrix(prim_a)

        # Clear the transform on the second prim
        xform = UsdGeom.Xformable(prim_b)
        xform.ClearXformOpOrder()

        # Set the pose of prim_b to that of prim_a
        xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op.Set(pose)

    def check_goal_trigger(self):
        """"""
        triggered = False
        triggerColliders = self.triggerStateAPI.GetTriggeredCollisionsRel().GetTargets()
        if len(triggerColliders) > 0:
            triggered = any(self.robot_root_path in path.pathString for path in triggerColliders)
        return triggered

    def space_occupied(self, point: Gf.Vec3f, ignore_base_paths: List[str] = []) -> bool:
        """"""
        # Get the PhysX Scene Query Interface
        scene_query_interface = get_physx_scene_query_interface()

        # Convert the Gf.Vec3f point to a carb.types.Float3 point
        carb_point = carb.Float3(point[0], point[1], point[2])

        # Define a small radius for the overlap sphere
        radius = 1.0

        # Define a report function for the overlap query
        def report_fn(hit):

            # Check if the USD path starts with any of the ignore base paths
            if any(hit.collision.startswith(base_path) for base_path in ignore_base_paths):
                # If the USD path starts with an ignore base path, ignore the hit
                return True

            # Otherwise, stop the overlap query
            return False

        # Perform an overlap sphere query at the point
        num_hits = scene_query_interface.overlap_sphere(radius, carb_point, report_fn)

        # If the overlap query returned any hits, the space is occupied
        return num_hits > 0

    def reset_goal_random(self, ground_plane_z=8, ground_plane_x_size=50, ground_plane_y_size=50):
        # Define the range of the ground plane
        x_range = (-ground_plane_x_size, ground_plane_x_size)
        y_range = (-ground_plane_y_size, ground_plane_y_size)
        z_range = (ground_plane_z, ground_plane_z)  # Assuming the ground plane is at z=ground_plane_z

        space_not_found = True
        random_pos: Gf.Vec3f
        while space_not_found:

            # Generate a random position within the range
            random_pos = Gf.Vec3f(
                random.uniform(*x_range),
                random.uniform(*y_range),
                random.uniform(*z_range)
            )
            space_not_found = self.space_occupied(random_pos,  ["/World/GroundPlane/CollisionPlane", "/World/goalTrigger"])
            
        # Set the goal to the random position
        self.init_goal_trigger(random_pos)
