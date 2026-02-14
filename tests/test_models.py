"""Unit tests for warehouse domain models."""

import pytest
from warehouse.models import (
    CellType,
    Position,
    Robot,
    RobotStatus,
    Task,
    TaskStatus,
    Warehouse,
)


class TestPosition:
    def test_manhattan_distance(self):
        a = Position(0, 0)
        b = Position(3, 4)
        assert a.manhattan_distance(b) == 7

    def test_manhattan_distance_symmetric(self):
        a = Position(1, 5)
        b = Position(4, 2)
        assert a.manhattan_distance(b) == b.manhattan_distance(a)

    def test_neighbors(self):
        p = Position(3, 3)
        nbrs = p.neighbors()
        assert len(nbrs) == 4
        assert Position(2, 3) in nbrs
        assert Position(4, 3) in nbrs
        assert Position(3, 2) in nbrs
        assert Position(3, 4) in nbrs

    def test_immutable(self):
        p = Position(1, 2)
        with pytest.raises(AttributeError):
            p.row = 5


class TestRobot:
    def test_initial_state(self):
        r = Robot(position=Position(0, 0))
        assert r.status == RobotStatus.IDLE
        assert r.battery == 100.0
        assert r.is_available

    def test_step_follows_path(self):
        r = Robot(position=Position(0, 0))
        r.path = [Position(0, 0), Position(0, 1), Position(0, 2)]
        r.path_index = 1
        new_pos = r.step()
        assert new_pos == Position(0, 1)
        assert r.position == Position(0, 1)

    def test_step_drains_battery(self):
        r = Robot(position=Position(0, 0), battery=50.0)
        r.path = [Position(0, 0), Position(0, 1)]
        r.path_index = 1
        r.step()
        assert r.battery < 50.0

    def test_not_available_when_busy(self):
        r = Robot(status=RobotStatus.EN_ROUTE_PICKUP)
        assert not r.is_available

    def test_not_available_low_battery(self):
        r = Robot(battery=10.0)
        assert not r.is_available


class TestWarehouse:
    def test_default_grid_all_floor(self):
        wh = Warehouse(rows=5, cols=5)
        for r in range(5):
            for c in range(5):
                assert wh.grid[r][c] == CellType.FLOOR

    def test_in_bounds(self):
        wh = Warehouse(rows=10, cols=10)
        assert wh.in_bounds(Position(0, 0))
        assert wh.in_bounds(Position(9, 9))
        assert not wh.in_bounds(Position(-1, 0))
        assert not wh.in_bounds(Position(10, 5))

    def test_obstacle_not_walkable(self):
        wh = Warehouse(rows=5, cols=5)
        wh.set_cell(Position(2, 2), CellType.OBSTACLE)
        assert not wh.is_walkable(Position(2, 2))
        assert wh.is_walkable(Position(0, 0))

    def test_add_and_get_robot(self):
        wh = Warehouse(rows=5, cols=5)
        r = Robot(id="R01", position=Position(0, 0))
        wh.add_robot(r)
        assert wh.get_robot("R01") is r
        assert wh.get_robot("nope") is None

    def test_add_and_get_task(self):
        wh = Warehouse(rows=5, cols=5)
        t = Task(id="T01", pickup=Position(1, 1), dropoff=Position(3, 3))
        wh.add_task(t)
        assert wh.get_task("T01") is t
        assert t in wh.pending_tasks

    def test_to_dict_returns_valid_snapshot(self):
        wh = Warehouse(rows=3, cols=3)
        wh.add_robot(Robot(id="R01", position=Position(0, 0)))
        wh.add_task(Task(id="T01", pickup=Position(1, 1), dropoff=Position(2, 2)))
        d = wh.to_dict()
        assert d["rows"] == 3
        assert d["cols"] == 3
        assert len(d["robots"]) == 1
        assert len(d["tasks"]) == 1
        assert "stats" in d
