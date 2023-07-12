from omni.kit.scripting import BehaviorScript
from omni.isaac.dynamic_control import _dynamic_control
from omni.physx import acquire_physx_interface, get_physx_interface, get_physx_simulation_interface
from omni.physx.bindings._physx import SimulationEvent
from omni.physx.scripts.physicsUtils import PhysicsSchemaTools
from omni.usd import get_context
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema

import numpy as np

class EnvironmentController(BehaviorScript):
    def on_init(self):
        self.robot_root_path = "/World/Mechanum_Robot"
        self.goal_path = "/World/Goal"
        self.predefined_area = [(-10, -10), (10, 10)]  # Define the area where the goal can be placed

    def on_play(self):
        pass
        # self._contact_report_sub = get_physx_simulation_interface().subscribe_contact_report_events(self._on_contact_report_event)
        # self._stepping_sub = get_physx_interface().subscribe_physics_step_events(self._on_physics_step)
        # contactReportAPI = PhysxSchema.PhysxContactReportAPI.Apply(goal_prime)
        # contactReportAPI.CreateThresholdAttr().Set(20000)

    def on_update(self, current_time, delta_time):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.stage = get_context().get_stage()
        self.goal_prime = self.stage.GetPrimAtPath(self.goal_path)
        self.check_collision(self.goal_prime, self.robot_root_path)
        # self.robot_actor = self.dc.get_rigid_body(self.robot_root_path)
        # self.goal_actor = self.dc.get_rigid_body(self.goal_path)
        
    def on_collision(self):
        # This function is called when the robot and the goal collide
        print("Collision detected")
        self.reset_goal_position()

    def reset_goal_position(self):
        # Generate a new random position within the predefined area
        new_position = np.random.uniform(self.predefined_area[0], self.predefined_area[1])
        self.dc.set_rigid_body_pose(self.goal_actor, new_position, [0, 0, 0, 1])

    def check_collision(self, prim, target_path: str):
        """
        Check for collision of given root prim for self-collision, collision with ground, collision with other collision objects
        in a world

        args:
            prim (Usd.Prim): Root prim for the collision check
            target_path (str): Path of the already placed xform prim where the asset is added

        return: 
            bool[self-collision], bool[collision with ground], bool[collision with other objects], List[objects path in collision]
        """
        self.self_collision, self.ground_collision, self.object_collision = \
            False, False, False
        self.overlaps = []
        self.collision_root_path = target_path
        prim = prim
        import omni.physx
        interface = omni.physx.get_physx_scene_query_interface()
        meshes = []
        self.get_collision_check_meshes(prim, meshes)

        # Iterate on all meshes in the added prim
        for mesh_path in meshes:
            # Get the encoded path
            path_id_0, path_id_1 = PhysicsSchemaTools.encodeSdfPath(mesh_path)
            # Look for collisions. We cannot use the direct hit count beca>
            interface.overlap_mesh(path_id_0, path_id_1, self.on_hit, False)
            #if len(self.overlaps) > 0:
            #    prim.GetReferences().ClearReferences()
            #    break
        # Return detected overlaps
        return {"self collision": self.self_collision, "ground collision": self.ground_collision, "object collision": self.object_collision, "overlaps": self.overlaps}

    def on_hit(self, hit) -> bool:
        """
        Callback method for checking a given mesh for collisions
        Args:
            hit (SceneQueryHits): Structure containing information about collision hits
        Returns:
            bool: True to continue scanning for collisions
        """
        # Check that we are not hitting the ground plane (this is normal), and not having self collision
        if ("ground_plane" in hit.rigid_body or "defaultGroundPlane"):
            self.ground_collision = True
        if(self.collision_root_path in hit.rigid_body):
            self.self_collision = True
        if (hit.rigid_body not in self.overlaps):
            self.overlaps.append(hit.rigid_body)
        return True

    def get_collision_check_meshes(self, prim: Usd.Prim, meshes: List[str]) -> None:
        """
        Get paths to children of a prim to which collision can be added. Temporarily removes instancing
        to find subprims that are meshes.
        Args:
            prim (Usd.Prim): Root prim for the collision check
            meshes (List[str]): List of children that are UsdGeomMesh for collision check. Includes
                                subprims of instanceable prims.
        """
        instanceables = []
        for child in prim.GetChildren():
            # If the prim is instanceable, temporarily make it not instanced to get subprims.
            if child.IsInstanceable():
                child.SetInstanceable(False)
                instanceables.append(child)
            # Select only meshes
            if child.IsA(UsdGeom.Mesh):
                meshes.append(child.GetPath().pathString)
                continue
            # Recursively call the method on subprims
            self.get_collision_check_meshes(child, meshes)
        for modified_prim in instanceables:
            modified_prim.SetInstanceable(True)

    def _on_physics_step(self, dt):
        contact_headers, contact_data = get_physx_simulation_interface().get_contact_report()
        for contact_header in contact_headers:
            print("Got contact header type: " + str(contact_header.type))
            print("Actor0: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.actor0)))
            print("Actor1: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.actor1)))
            print("Collider0: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.collider0)))
            print("Collider1: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.collider1)))
            print("StageId: " + str(contact_header.stage_id))
            print("Number of contacts: " + str(contact_header.num_contact_data))
            
            contact_data_offset = contact_header.contact_data_offset
            num_contact_data = contact_header.num_contact_data
            
            for index in range(contact_data_offset, contact_data_offset + num_contact_data, 1):
                print("Contact:")
                print("Contact position: " + str(contact_data[index].position))
                print("Contact normal: " + str(contact_data[index].normal))
                print("Contact impulse: " + str(contact_data[index].impulse))
                print("Contact separation: " + str(contact_data[index].separation))
                print("Contact faceIndex0: " + str(contact_data[index].face_index0))
                print("Contact faceIndex1: " + str(contact_data[index].face_index1))
                print("Contact material0: " + str(PhysicsSchemaTools.intToSdfPath(contact_data[index].material0)))
                print("Contact material1: " + str(PhysicsSchemaTools.intToSdfPath(contact_data[index].material1)))

    def _on_contact_report_event(self, contact_headers, contact_data):
        for contact_header in contact_headers:
            print("Got contact header type: " + str(contact_header.type))
            print("Actor0: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.actor0)))
            print("Actor1: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.actor1)))
            print("Collider0: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.collider0)))
            print("Collider1: " + str(PhysicsSchemaTools.intToSdfPath(contact_header.collider1)))
            print("StageId: " + str(contact_header.stage_id))
            print("Number of contacts: " + str(contact_header.num_contact_data))
            
            contact_data_offset = contact_header.contact_data_offset
            num_contact_data = contact_header.num_contact_data
            
            for index in range(contact_data_offset, contact_data_offset + num_contact_data, 1):
                print("Contact:")
                print("Contact position: " + str(contact_data[index].position))
                print("Contact normal: " + str(contact_data[index].normal))
                print("Contact impulse: " + str(contact_data[index].impulse))
                print("Contact separation: " + str(contact_data[index].separation))
                print("Contact faceIndex0: " + str(contact_data[index].face_index0))
                print("Contact faceIndex1: " + str(contact_data[index].face_index1))
                print("Contact material0: " + str(PhysicsSchemaTools.intToSdfPath(contact_data[index].material0)))
                print("Contact material1: " + str(PhysicsSchemaTools.intToSdfPath(contact_data[index].material1)))
    