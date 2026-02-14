"""Unit tests for pathfinding algorithms (A* and CBS)."""

import pytest
from warehouse.models import CellType, Position, Warehouse
from warehouse.pathfinding import AStarPlanner, ConflictBasedSearch


@pytest.fixture
def simple_warehouse():
    """5x5 open warehouse with no obstacles."""
    return Warehouse(rows=5, cols=5)


@pytest.fixture
def obstacle_warehouse():
    """5x5 warehouse with a wall in the middle row, gap at col 4."""
    wh = Warehouse(rows=5, cols=5)
    for c in range(4):
        wh.set_cell(Position(2, c), CellType.OBSTACLE)
    return wh


class TestAStar:
    def test_straight_path(self, simple_warehouse):
        planner = AStarPlanner(simple_warehouse)
        path = planner.plan(Position(0, 0), Position(0, 4))
        assert path is not None
        assert path[0] == Position(0, 0)
        assert path[-1] == Position(0, 4)
        assert len(path) == 5  # optimal

    def test_same_start_goal(self, simple_warehouse):
        planner = AStarPlanner(simple_warehouse)
        path = planner.plan(Position(2, 2), Position(2, 2))
        assert path == [Position(2, 2)]

    def test_path_around_obstacle(self, obstacle_warehouse):
        planner = AStarPlanner(obstacle_warehouse)
        path = planner.plan(Position(0, 0), Position(4, 0))
        assert path is not None
        assert path[0] == Position(0, 0)
        assert path[-1] == Position(4, 0)
        # Path must not go through obstacles
        for p in path:
            assert obstacle_warehouse.is_walkable(p)

    def test_no_path_exists(self):
        """Fully blocked destination — surrounded by obstacles."""
        wh = Warehouse(rows=3, cols=3)
        # Surround (2,2) with obstacles on all walkable sides
        for r, c in [(1, 2), (2, 1)]:
            wh.set_cell(Position(r, c), CellType.OBSTACLE)
        planner = AStarPlanner(wh)
        path = planner.plan(Position(0, 0), Position(2, 2))
        # (2,2) is completely blocked (only reachable via (1,2) and (2,1), both obstacles)
        assert path is None

    def test_respects_constraints(self, simple_warehouse):
        planner = AStarPlanner(simple_warehouse)
        # Block position (0,1) at timestep 1
        constraints = {(0, 1, 1)}
        path = planner.plan(Position(0, 0), Position(0, 2), constraints=constraints)
        assert path is not None
        assert path[-1] == Position(0, 2)
        # At timestep 1, robot should NOT be at (0,1)
        if len(path) > 1:
            assert path[1] != Position(0, 1)


class TestCBS:
    def test_two_agents_no_conflict(self, simple_warehouse):
        cbs = ConflictBasedSearch(simple_warehouse)
        requests = [
            (Position(0, 0), Position(0, 4)),
            (Position(4, 0), Position(4, 4)),
        ]
        result = cbs.solve(requests)
        assert result is not None
        assert len(result) == 2
        # Both reach their goals
        assert result[0][-1] == Position(0, 4)
        assert result[1][-1] == Position(4, 4)

    def test_two_agents_head_on(self, simple_warehouse):
        """Two agents on collision course — CBS must resolve."""
        cbs = ConflictBasedSearch(simple_warehouse)
        requests = [
            (Position(2, 0), Position(2, 4)),
            (Position(2, 4), Position(2, 0)),
        ]
        result = cbs.solve(requests)
        assert result is not None
        # Verify no vertex collisions
        max_t = max(len(result[0]), len(result[1]))
        for t in range(max_t):
            p0 = result[0][min(t, len(result[0]) - 1)]
            p1 = result[1][min(t, len(result[1]) - 1)]
            assert p0 != p1, f"Collision at timestep {t}: {p0}"

    def test_empty_requests(self, simple_warehouse):
        cbs = ConflictBasedSearch(simple_warehouse)
        result = cbs.solve([])
        assert result == {}

    def test_three_agents(self, simple_warehouse):
        cbs = ConflictBasedSearch(simple_warehouse)
        requests = [
            (Position(0, 0), Position(4, 4)),
            (Position(4, 0), Position(0, 4)),
            (Position(0, 4), Position(4, 0)),
        ]
        result = cbs.solve(requests)
        assert result is not None
        assert len(result) == 3
        # All reach their goals
        for i, (_, goal) in enumerate(requests):
            assert result[i][-1] == goal
