"""Unit tests for the task allocation engine."""

import pytest
from warehouse.models import Position, Robot, Task, Warehouse
from warehouse.task_allocator import TaskAllocator


@pytest.fixture
def warehouse_with_robots_and_tasks():
    wh = Warehouse(rows=10, cols=10)

    # Two robots at different positions
    r1 = Robot(id="R01", position=Position(0, 0), battery=90.0)
    r2 = Robot(id="R02", position=Position(9, 9), battery=80.0)
    wh.add_robot(r1)
    wh.add_robot(r2)

    # Two tasks — one near each robot
    t1 = Task(id="T01", pickup=Position(1, 1), dropoff=Position(5, 5), priority=3)
    t2 = Task(id="T02", pickup=Position(8, 8), dropoff=Position(4, 4), priority=5)
    wh.add_task(t1)
    wh.add_task(t2)

    return wh


class TestTaskAllocator:
    def test_allocates_nearest(self, warehouse_with_robots_and_tasks):
        wh = warehouse_with_robots_and_tasks
        allocator = TaskAllocator(wh)
        assignments = allocator.allocate()

        assert len(assignments) == 2
        ids = {rid: tid for rid, tid in assignments}
        # R01 (at 0,0) should get T01 (pickup 1,1) — much closer
        assert ids["R01"] == "T01"
        # R02 (at 9,9) should get T02 (pickup 8,8) — much closer
        assert ids["R02"] == "T02"

    def test_no_tasks(self):
        wh = Warehouse(rows=5, cols=5)
        wh.add_robot(Robot(id="R01", position=Position(0, 0)))
        allocator = TaskAllocator(wh)
        assert allocator.allocate() == []

    def test_no_robots(self):
        wh = Warehouse(rows=5, cols=5)
        wh.add_task(Task(pickup=Position(1, 1), dropoff=Position(3, 3)))
        allocator = TaskAllocator(wh)
        assert allocator.allocate() == []

    def test_apply_assignments(self, warehouse_with_robots_and_tasks):
        wh = warehouse_with_robots_and_tasks
        allocator = TaskAllocator(wh)
        assignments = allocator.allocate()
        allocator.apply_assignments(assignments)

        # Robots should now be en-route
        from warehouse.models import RobotStatus, TaskStatus
        for robot in wh.robots.values():
            assert robot.status == RobotStatus.EN_ROUTE_PICKUP
        for task in wh.tasks.values():
            assert task.status == TaskStatus.ASSIGNED

    def test_priority_influences_assignment(self):
        """High-priority task should be preferred even if slightly farther."""
        wh = Warehouse(rows=10, cols=10)
        r1 = Robot(id="R01", position=Position(5, 5), battery=90.0)
        wh.add_robot(r1)

        # Low-priority close task vs high-priority slightly farther
        t_low = Task(id="TL", pickup=Position(5, 6), dropoff=Position(0, 0), priority=1)
        t_high = Task(id="TH", pickup=Position(5, 7), dropoff=Position(0, 0), priority=5)
        wh.add_task(t_low)
        wh.add_task(t_high)

        allocator = TaskAllocator(wh)
        assignments = allocator.allocate()
        assert len(assignments) == 1
        # Should pick high-priority task
        assert assignments[0][1] == "TH"
