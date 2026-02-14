"""
Path-planning algorithms for warehouse robots.

Implements:
  - A* search for single-agent shortest paths.
  - Conflict-Based Search (CBS) for optimal multi-agent path finding
    that avoids inter-robot collisions in both space and time.

Reference:
  Sharon et al., "Conflict-Based Search for Optimal Multi-Agent
  Pathfinding", Artificial Intelligence, 2015.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field
from typing import Optional

from warehouse.models import Position, Warehouse


# ---------------------------------------------------------------------------
# A* single-agent planner
# ---------------------------------------------------------------------------

class AStarPlanner:
    """
    Time-aware A* planner on a 4-connected grid.

    Supports *constraint tables* so it can be used as the low-level
    solver inside CBS.  A constraint ``(row, col, t)`` forbids the
    agent from occupying that cell at timestep *t*.
    """

    def __init__(self, warehouse: Warehouse) -> None:
        self._wh = warehouse

    def plan(
        self,
        start: Position,
        goal: Position,
        constraints: Optional[set[tuple[int, int, int]]] = None,
        max_timesteps: int = 200,
    ) -> Optional[list[Position]]:
        """
        Return a time-indexed path from *start* to *goal*, or ``None``
        if no path exists within *max_timesteps*.

        Each position in the returned list corresponds to one timestep.
        """
        if start == goal:
            return [start]

        constraints = constraints or set()

        # (f, g, timestep, position)  — tie-break on g then position
        open_set: list[tuple[int, int, int, Position]] = []
        heapq.heappush(open_set, (start.manhattan_distance(goal), 0, 0, start))

        # best g-value seen at (position, timestep)
        best: dict[tuple[Position, int], int] = {(start, 0): 0}
        parent: dict[tuple[Position, int], tuple[Position, int]] = {}

        while open_set:
            f, g, t, pos = heapq.heappop(open_set)

            if pos == goal:
                return self._reconstruct(parent, start, pos, t)

            if t >= max_timesteps:
                continue

            nt = t + 1
            # successors: 4 neighbors + wait-in-place
            for npos in self._wh.walkable_neighbors(pos) + [pos]:
                if (npos.row, npos.col, nt) in constraints:
                    continue

                ng = g + 1
                key = (npos, nt)
                if key not in best or ng < best[key]:
                    best[key] = ng
                    parent[key] = (pos, t)
                    h = npos.manhattan_distance(goal)
                    heapq.heappush(open_set, (ng + h, ng, nt, npos))

        return None  # no path found

    @staticmethod
    def _reconstruct(
        parent: dict[tuple[Position, int], tuple[Position, int]],
        start: Position,
        goal: Position,
        goal_t: int,
    ) -> list[Position]:
        path: list[Position] = []
        state = (goal, goal_t)
        while state[0] != start or state[1] != 0:
            path.append(state[0])
            state = parent[state]
        path.append(start)
        path.reverse()
        return path


# ---------------------------------------------------------------------------
# Conflict-Based Search (CBS) — multi-agent planner
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Conflict:
    """A collision between two agents at a given position and timestep."""
    agent_a: int
    agent_b: int
    position: Position
    timestep: int


@dataclass(order=True)
class CBSNode:
    """Node in the CBS constraint tree."""
    cost: int
    constraints: dict[int, set[tuple[int, int, int]]] = field(
        default_factory=dict, compare=False
    )
    paths: dict[int, list[Position]] = field(
        default_factory=dict, compare=False
    )


class ConflictBasedSearch:
    """
    Optimal multi-agent path finder using CBS.

    Given a list of (start, goal) pairs, returns collision-free paths
    for all agents simultaneously.
    """

    def __init__(self, warehouse: Warehouse) -> None:
        self._wh = warehouse
        self._planner = AStarPlanner(warehouse)

    def solve(
        self,
        requests: list[tuple[Position, Position]],
        max_iterations: int = 500,
    ) -> Optional[dict[int, list[Position]]]:
        """
        Solve multi-agent pathfinding.

        Parameters
        ----------
        requests : list of (start, goal) tuples, one per agent.
        max_iterations : search budget.

        Returns
        -------
        dict mapping agent index -> path, or None if unsolvable.
        """
        if not requests:
            return {}

        # Build root node — plan each agent independently
        root = CBSNode(cost=0)
        for i, (start, goal) in enumerate(requests):
            path = self._planner.plan(start, goal)
            if path is None:
                return None
            root.paths[i] = path
            root.constraints[i] = set()
        root.cost = self._sum_of_costs(root.paths)

        open_list: list[CBSNode] = [root]
        iterations = 0

        while open_list and iterations < max_iterations:
            iterations += 1
            node = heapq.heappop(open_list)

            conflict = self._first_conflict(node.paths)
            if conflict is None:
                return node.paths  # solution found

            # Branch: add constraint for each conflicting agent
            for agent in (conflict.agent_a, conflict.agent_b):
                child = CBSNode(cost=0)
                # deep copy constraints
                child.constraints = {
                    k: set(v) for k, v in node.constraints.items()
                }
                child.paths = dict(node.paths)

                child.constraints.setdefault(agent, set()).add(
                    (conflict.position.row, conflict.position.col, conflict.timestep)
                )

                new_path = self._planner.plan(
                    requests[agent][0],
                    requests[agent][1],
                    constraints=child.constraints[agent],
                )
                if new_path is None:
                    continue  # prune
                child.paths[agent] = new_path
                child.cost = self._sum_of_costs(child.paths)
                heapq.heappush(open_list, child)

        return None  # exceeded budget

    # -- helpers ------------------------------------------------------------

    @staticmethod
    def _sum_of_costs(paths: dict[int, list[Position]]) -> int:
        return sum(len(p) for p in paths.values())

    @staticmethod
    def _first_conflict(paths: dict[int, list[Position]]) -> Optional[Conflict]:
        """Detect the first vertex conflict across all agents."""
        max_t = max((len(p) for p in paths.values()), default=0)
        agents = sorted(paths.keys())

        for t in range(max_t):
            occupied: dict[Position, int] = {}
            for a in agents:
                pos = paths[a][min(t, len(paths[a]) - 1)]
                if pos in occupied:
                    return Conflict(
                        agent_a=occupied[pos],
                        agent_b=a,
                        position=pos,
                        timestep=t,
                    )
                occupied[pos] = a
        return None
