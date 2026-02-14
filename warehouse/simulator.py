"""
Discrete-event simulation engine for warehouse robotics.

Orchestrates the full loop:
  1. Accept new tasks
  2. Allocate tasks to robots (Hungarian algorithm)
  3. Plan collision-free paths (CBS / A*)
  4. Step all robots forward one tick
  5. Handle pickups, dropoffs, and completion
"""

from __future__ import annotations

import logging
import random
from typing import Optional

from warehouse.models import (
    CellType,
    Position,
    Robot,
    RobotStatus,
    Task,
    TaskStatus,
    Warehouse,
)
from warehouse.pathfinding import AStarPlanner, ConflictBasedSearch
from warehouse.task_allocator import TaskAllocator

logger = logging.getLogger(__name__)


class Simulator:
    """
    Main simulation controller.

    Call :meth:`step` to advance the simulation by one tick, or
    :meth:`run` to execute multiple ticks.
    """

    def __init__(self, warehouse: Warehouse, use_cbs: bool = True) -> None:
        self._wh = warehouse
        self._allocator = TaskAllocator(warehouse)
        self._planner = AStarPlanner(warehouse)
        self._cbs = ConflictBasedSearch(warehouse) if use_cbs else None
        self._use_cbs = use_cbs

    @property
    def warehouse(self) -> Warehouse:
        return self._wh

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(self) -> dict:
        """
        Advance the simulation by one discrete timestep.

        Returns a summary dict of what happened this tick.
        """
        events: list[str] = []

        # 1. Allocate pending tasks to idle robots
        assignments = self._allocator.allocate()
        if assignments:
            self._allocator.apply_assignments(assignments)
            events.append(f"assigned {len(assignments)} task(s)")

        # 2. Plan paths for newly assigned robots
        self._plan_paths(events)

        # 3. Move robots one step along their paths
        self._move_robots(events)

        # 4. Handle arrivals (pickups and dropoffs)
        self._handle_arrivals(events)

        self._wh.tick += 1

        return {
            "tick": self._wh.tick,
            "events": events,
            "stats": self._wh.to_dict()["stats"],
        }

    def run(self, ticks: int = 100) -> list[dict]:
        """Run the simulation for *ticks* steps and return all summaries."""
        return [self.step() for _ in range(ticks)]

    def add_random_task(self) -> Task:
        """Create a random pick-and-place task on walkable cells."""
        walkable = self._get_walkable_positions()
        pickup = random.choice(walkable)
        dropoff = random.choice(walkable)
        while dropoff == pickup:
            dropoff = random.choice(walkable)

        task = Task(
            pickup=pickup,
            dropoff=dropoff,
            priority=random.randint(1, 5),
        )
        self._wh.add_task(task)
        return task

    # ------------------------------------------------------------------
    # Class method for quick demo setup
    # ------------------------------------------------------------------

    @classmethod
    def create_demo(
        cls,
        rows: int = 20,
        cols: int = 30,
        n_robots: int = 5,
        n_tasks: int = 10,
        obstacle_density: float = 0.15,
        seed: int = 42,
    ) -> "Simulator":
        """
        Create a fully-configured demo warehouse.

        Generates a grid with random obstacles, shelf aisles,
        stations, and places robots and tasks.
        """
        random.seed(seed)
        wh = Warehouse(rows=rows, cols=cols)

        # Place obstacles to form aisle structure
        for r in range(rows):
            for c in range(cols):
                if random.random() < obstacle_density:
                    # Keep borders clear for robot movement
                    if r in (0, rows - 1) or c in (0, cols - 1):
                        continue
                    wh.set_cell(Position(r, c), CellType.OBSTACLE)

        # Place some shelves
        for r in range(2, rows - 2, 3):
            for c in range(2, cols - 2, 4):
                if wh.grid[r][c] == CellType.FLOOR:
                    wh.set_cell(Position(r, c), CellType.SHELF)

        # Place stations along bottom row
        for c in range(0, cols, 5):
            wh.set_cell(Position(rows - 1, c), CellType.STATION)

        # Place charger at top-left
        wh.set_cell(Position(0, 0), CellType.CHARGER)

        # Spawn robots on walkable cells along top row
        walkable_top = [
            Position(0, c) for c in range(cols)
            if wh.grid[0][c] != CellType.OBSTACLE
        ]
        for i in range(min(n_robots, len(walkable_top))):
            robot = Robot(
                id=f"R{i:02d}",
                position=walkable_top[i % len(walkable_top)],
                battery=round(random.uniform(60.0, 100.0), 1),
            )
            wh.add_robot(robot)

        sim = cls(wh)

        # Generate initial tasks
        for _ in range(n_tasks):
            sim.add_random_task()

        return sim

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _plan_paths(self, events: list[str]) -> None:
        """Plan paths for robots that need them."""
        robots_needing_paths: list[Robot] = []
        goals: list[Position] = []

        for robot in self._wh.robots.values():
            task = self._get_robot_task(robot)
            if task is None:
                continue

            goal: Optional[Position] = None
            if robot.status == RobotStatus.EN_ROUTE_PICKUP:
                goal = task.pickup
            elif robot.status == RobotStatus.EN_ROUTE_DROPOFF:
                goal = task.dropoff

            if goal is not None and not robot.path:
                robots_needing_paths.append(robot)
                goals.append(goal)

        if not robots_needing_paths:
            return

        if self._use_cbs and self._cbs and len(robots_needing_paths) > 1:
            requests = [
                (r.position, g)
                for r, g in zip(robots_needing_paths, goals)
            ]
            result = self._cbs.solve(requests)
            if result:
                for idx, robot in enumerate(robots_needing_paths):
                    robot.path = result[idx]
                    robot.path_index = 1  # skip start position
                events.append(f"CBS planned {len(robots_needing_paths)} path(s)")
                return

        # Fallback: plan individually with A*
        for robot, goal in zip(robots_needing_paths, goals):
            path = self._planner.plan(robot.position, goal)
            if path:
                robot.path = path
                robot.path_index = 1
            else:
                logger.warning("No path for robot %s to %s", robot.id, goal)
        events.append(f"A* planned {len(robots_needing_paths)} path(s)")

    def _move_robots(self, events: list[str]) -> None:
        """Advance each robot one step along its path."""
        moved = 0
        for robot in self._wh.robots.values():
            if robot.path and robot.path_index < len(robot.path):
                robot.step()
                moved += 1
        if moved:
            events.append(f"{moved} robot(s) moved")

    def _handle_arrivals(self, events: list[str]) -> None:
        """Handle robots that have reached their destination."""
        for robot in self._wh.robots.values():
            if not robot.has_arrived:
                continue

            task = self._get_robot_task(robot)
            if task is None:
                continue

            if robot.status == RobotStatus.EN_ROUTE_PICKUP:
                # Arrived at pickup — start carrying
                robot.status = RobotStatus.EN_ROUTE_DROPOFF
                task.status = TaskStatus.IN_PROGRESS
                robot.path = []
                robot.path_index = 0
                events.append(f"{robot.id} picked up {task.id}")

            elif robot.status == RobotStatus.EN_ROUTE_DROPOFF:
                # Arrived at dropoff — task complete
                robot.status = RobotStatus.IDLE
                robot.carrying_task = None
                robot.path = []
                robot.path_index = 0
                robot.tasks_completed += 1
                task.status = TaskStatus.COMPLETED
                task.completed_tick = self._wh.tick
                self._wh.completed_tasks.append(task.id)
                events.append(f"{robot.id} completed {task.id}")

    def _get_robot_task(self, robot: Robot) -> Optional[Task]:
        if robot.carrying_task is None:
            return None
        return self._wh.get_task(robot.carrying_task)

    def _get_walkable_positions(self) -> list[Position]:
        return [
            Position(r, c)
            for r in range(self._wh.rows)
            for c in range(self._wh.cols)
            if self._wh.is_walkable(Position(r, c))
        ]
