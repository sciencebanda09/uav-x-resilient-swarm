import numpy as np
from uav_x.core.obstacles import ObstacleField


def test_segment_intersection_and_detour():
    field = ObstacleField.from_json("scene/obstacles.json")
    hit = field.blocking(np.array([180., 260., 18.]), np.array([260., 260., 18.]))
    assert hit is not None and hit.obstacle_id == "building-01"
    detour = field.detour(np.array([180., 260., 18.]), np.array([260., 260., 18.]))
    assert detour is not None and detour[0][2] > hit.hi[2]
