import numpy as np
from uav_x.core.config import SeedManager, config_hash
from uav_x.core.environment import TerrainModel, WeatherModel
from uav_x.core.transport import DelayedPacketQueue
from uav_x.simulation.disturbances import Disturbance, DisturbanceScheduler
from uav_x.core.models import ScenarioConfig

def test_seed_and_config_hash_are_stable():
    assert np.array_equal(SeedManager(4).get("x").normal(size=3), SeedManager(4).get("x").normal(size=3))
    assert config_hash(ScenarioConfig(seed=4)) == config_hash(ScenarioConfig(seed=4))

def test_environment_and_transport():
    terrain = TerrainModel(500, seed=4); assert terrain.height_at(10, 10) >= 0
    weather = WeatherModel(4).step(12); assert weather.visibility <= 1.0
    queue = DelayedPacketQueue(4); assert queue.send("UAV-1", {}, 0, 10, 0)
    assert queue.drain(.005) == []; assert len(queue.drain(.02)) == 1

def test_disturbance_scheduler_fires_once_in_time_order():
    scheduler = DisturbanceScheduler([Disturbance(3, "failure", {}), Disturbance(1, "outage", {})])
    assert [x.kind for x in scheduler.due(2)] == ["outage"]
    assert [x.kind for x in scheduler.due(4)] == ["failure"]
    assert scheduler.due(5) == []
