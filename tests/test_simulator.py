"""Unit tests for the simulation engine."""

import pytest
from warehouse.models import Position, Robot, RobotStatus, Task, TaskStatus, Warehouse
from warehouse.simulator import Simulator


class TestSimulator:
    def test_create_demo(self):
        sim = Simulator.create_demo(rows=10, cols=10, n_robots=3, n_tasks=5)
        wh = sim.warehouse
        assert wh.rows == 10
        assert wh.cols == 10
        assert len(wh.robots) == 3
        assert len(wh.tasks) == 5

    def test_single_step(self):
        sim = Simulator.create_demo(rows=10, cols=10, n_robots=2, n_tasks=2, seed=1)
        result = sim.step()
        assert "tick" in result
        assert "events" in result
        assert "stats" in result
        assert result["tick"] == 1

    def test_run_multiple_ticks(self):
        sim = Simulator.create_demo(rows=10, cols=10, n_robots=2, n_tasks=2, seed=1)
        results = sim.run(ticks=20)
        assert len(results) == 20
        assert sim.warehouse.tick == 20

    def test_tasks_eventually_complete(self):
        """With enough ticks, at least some tasks should complete."""
        sim = Simulator.create_demo(
            rows=8, cols=8, n_robots=3, n_tasks=2,
            obstacle_density=0.0, seed=42,
        )
        sim.run(ticks=100)
        completed = sum(
            1 for t in sim.warehouse.tasks.values()
            if t.status == TaskStatus.COMPLETED
        )
        assert completed > 0, "Expected at least one task to complete"

    def test_add_random_task(self):
        sim = Simulator.create_demo(rows=5, cols=5, n_robots=1, n_tasks=0)
        assert len(sim.warehouse.tasks) == 0
        task = sim.add_random_task()
        assert len(sim.warehouse.tasks) == 1
        assert sim.warehouse.is_walkable(task.pickup)
        assert sim.warehouse.is_walkable(task.dropoff)

    def test_no_robots_overlap_after_step(self):
        """After stepping, no two robots should share the same cell."""
        sim = Simulator.create_demo(
            rows=10, cols=10, n_robots=4, n_tasks=4,
            obstacle_density=0.0, seed=7,
        )
        for _ in range(30):
            sim.step()
            positions = [r.position for r in sim.warehouse.robots.values()]
            assert len(positions) == len(set(positions)), (
                f"Robot collision detected at tick {sim.warehouse.tick}"
            )
