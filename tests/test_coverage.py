from uav_x.simulation.coverage import CoverageGrid

def test_coverage_grid_marks_and_deduplicates_camera_footprints():
    grid = CoverageGrid(100, cell_size_m=10)
    assert grid.fraction == 0.0
    first = grid.mark_footprint(50, 50, 15)
    second = grid.mark_footprint(50, 50, 15)
    assert first > 0
    assert second == 0
    assert 0.0 < grid.fraction < 1.0
    assert grid.coverage_at(50, 50)
