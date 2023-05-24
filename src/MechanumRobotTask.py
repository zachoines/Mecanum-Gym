from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.viewports import set_camera_view

import omni.kit

from gym import spaces
import numpy as np
import torch
import math


class MechanumRobotTask(BaseTask):
    def __init__(self, name, offset=None) -> None:
        self._robot_position = [0.0, 0.0, 0.0]

        BaseTask.__init__(self, name=name, offset=offset)

    def set_up_scene(self, scene) -> None:
        pass

    
