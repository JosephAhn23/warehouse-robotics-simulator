"""
Task allocation engine using the Hungarian algorithm.

Assigns pending tasks to available robots by minimizing total
weighted cost, which combines Manhattan distance, task priority,
and robot battery level.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment

from warehouse.models import (
    Position,
    Robot,
    RobotStatus,
    Task,
    TaskStatus,
    Warehouse,
)


class TaskAllocator:
    """
    Centralized task-to-robot assignment optimizer.

    Uses the Hungarian algorithm (Kuhn-Munkres) to compute the
    minimum-cost bipartite matching between available robots and
    pending tasks each allocation cycle.
    """

    # Weights for the cost function
    DISTANCE_WEIGHT = 1.0
    PRIORITY_WEIGHT = 5.0
    BATTERY_PENALTY_WEIGHT = 0.3

    def __init__(self, warehouse: Warehouse) -> None:
        self._wh = warehouse

    def allocate(self) -> list[tuple[str, str]]:
        """
        Compute optimal robot-task assignments.

        Returns
        -------
        list of (robot_id, task_id) pairs representing new assignments.
        """
        robots = self._wh.available_robots
        tasks = self._wh.pending_tasks

        if not robots or not tasks:
            return []

        cost_matrix = self._build_cost_matrix(robots, tasks)

        # Hungarian algorithm — scipy expects (n, m) matrix
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        assignments: list[tuple[str, str]] = []
        for r_idx, t_idx in zip(row_ind, col_ind):
            if r_idx >= len(robots) or t_idx >= len(tasks):
                continue
            # Skip if cost is prohibitively high (robot can't reach task)
            if cost_matrix[r_idx][t_idx] >= 1e6:
                continue

            robot = robots[r_idx]
            task = tasks[t_idx]
            assignments.append((robot.id, task.id))

        return assignments

    def _build_cost_matrix(
        self,
        robots: list[Robot],
        tasks: list[Task],
    ) -> np.ndarray:
        """
        Build the cost matrix C[i][j] = cost of assigning robot i to task j.

        Cost combines:
          - distance from robot to task pickup
          - inverse priority (high priority → lower cost)
          - battery penalty (low battery → higher cost)
        """
        n_robots = len(robots)
        n_tasks = len(tasks)
        # Pad to square if necessary
        size = max(n_robots, n_tasks)
        cost = np.full((size, size), 1e6, dtype=np.float64)

        for i, robot in enumerate(robots):
            for j, task in enumerate(tasks):
                dist = robot.position.manhattan_distance(task.pickup)
                priority_bonus = (6 - task.priority) * self.PRIORITY_WEIGHT
                battery_cost = max(0.0, (30.0 - robot.battery)) * self.BATTERY_PENALTY_WEIGHT

                cost[i][j] = (
                    dist * self.DISTANCE_WEIGHT
                    + priority_bonus
                    + battery_cost
                )

        return cost

    def apply_assignments(
        self,
        assignments: list[tuple[str, str]],
    ) -> None:
        """
        Apply computed assignments to the warehouse state.

        Sets robot status and task status accordingly.
        """
        for robot_id, task_id in assignments:
            robot = self._wh.get_robot(robot_id)
            task = self._wh.get_task(task_id)
            if robot is None or task is None:
                continue

            robot.status = RobotStatus.EN_ROUTE_PICKUP
            robot.carrying_task = task_id
            task.status = TaskStatus.ASSIGNED
            task.assigned_robot = robot_id
