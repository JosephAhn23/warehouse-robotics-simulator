"""
Core domain models for the warehouse robotics simulator.

Defines the warehouse grid, robots, tasks, and their states using
immutable value objects and clear state machines.
"""

from __future__ import annotations

import enum
import uuid
from dataclasses import dataclass, field
from typing import Optional


# ---------------------------------------------------------------------------
# Value objects
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Position:
    """Discrete (row, col) coordinate on the warehouse grid."""

    row: int
    col: int

    def manhattan_distance(self, other: Position) -> int:
        return abs(self.row - other.row) + abs(self.col - other.col)

    def neighbors(self) -> list[Position]:
        """Return 4-connected neighbors (up, down, left, right)."""
        return [
            Position(self.row - 1, self.col),
            Position(self.row + 1, self.col),
            Position(self.row, self.col - 1),
            Position(self.row, self.col + 1),
        ]

    def __lt__(self, other: Position) -> bool:
        return (self.row, self.col) < (other.row, other.col)


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class RobotStatus(str, enum.Enum):
    IDLE = "idle"
    EN_ROUTE_PICKUP = "en_route_pickup"
    PICKING = "picking"
    EN_ROUTE_DROPOFF = "en_route_dropoff"
    DROPPING = "dropping"
    CHARGING = "charging"


class TaskStatus(str, enum.Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


class CellType(str, enum.Enum):
    FLOOR = "floor"
    OBSTACLE = "obstacle"
    SHELF = "shelf"
    STATION = "station"       # pick / drop-off station
    CHARGER = "charger"


# ---------------------------------------------------------------------------
# Mutable domain entities
# ---------------------------------------------------------------------------

@dataclass
class Robot:
    """An autonomous mobile robot operating in the warehouse."""

    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    position: Position = field(default_factory=lambda: Position(0, 0))
    status: RobotStatus = RobotStatus.IDLE
    battery: float = 100.0          # percentage 0-100
    carrying_task: Optional[str] = None  # task id if carrying
    path: list[Position] = field(default_factory=list)
    path_index: int = 0
    tasks_completed: int = 0

    @property
    def is_available(self) -> bool:
        return self.status == RobotStatus.IDLE and self.battery > 15.0

    def step(self) -> Optional[Position]:
        """Advance one step along the current path. Returns new position or None."""
        if self.path_index < len(self.path):
            self.position = self.path[self.path_index]
            self.path_index += 1
            self.battery = max(0.0, self.battery - 0.1)
            return self.position
        return None

    @property
    def has_arrived(self) -> bool:
        return self.path_index >= len(self.path) and len(self.path) > 0


@dataclass
class Task:
    """A pick-and-place job: move an item from pickup to dropoff."""

    id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    pickup: Position = field(default_factory=lambda: Position(0, 0))
    dropoff: Position = field(default_factory=lambda: Position(0, 0))
    status: TaskStatus = TaskStatus.PENDING
    priority: int = 1                # 1 (low) – 5 (critical)
    assigned_robot: Optional[str] = None
    created_tick: int = 0
    completed_tick: Optional[int] = None

    @property
    def is_assignable(self) -> bool:
        return self.status == TaskStatus.PENDING


# ---------------------------------------------------------------------------
# Warehouse environment
# ---------------------------------------------------------------------------

@dataclass
class Warehouse:
    """Grid-based warehouse environment with obstacles, shelves, and stations."""

    rows: int
    cols: int
    grid: list[list[CellType]] = field(default_factory=list)
    robots: dict[str, Robot] = field(default_factory=dict)
    tasks: dict[str, Task] = field(default_factory=dict)
    completed_tasks: list[str] = field(default_factory=list)
    tick: int = 0

    def __post_init__(self) -> None:
        if not self.grid:
            self.grid = [
                [CellType.FLOOR for _ in range(self.cols)]
                for _ in range(self.rows)
            ]

    # -- grid helpers -------------------------------------------------------

    def in_bounds(self, pos: Position) -> bool:
        return 0 <= pos.row < self.rows and 0 <= pos.col < self.cols

    def is_walkable(self, pos: Position) -> bool:
        if not self.in_bounds(pos):
            return False
        return self.grid[pos.row][pos.col] != CellType.OBSTACLE

    def set_cell(self, pos: Position, cell_type: CellType) -> None:
        if self.in_bounds(pos):
            self.grid[pos.row][pos.col] = cell_type

    def walkable_neighbors(self, pos: Position) -> list[Position]:
        return [n for n in pos.neighbors() if self.is_walkable(n)]

    # -- entity management --------------------------------------------------

    def add_robot(self, robot: Robot) -> None:
        self.robots[robot.id] = robot

    def add_task(self, task: Task) -> None:
        task.created_tick = self.tick
        self.tasks[task.id] = task

    def get_robot(self, robot_id: str) -> Optional[Robot]:
        return self.robots.get(robot_id)

    def get_task(self, task_id: str) -> Optional[Task]:
        return self.tasks.get(task_id)

    @property
    def pending_tasks(self) -> list[Task]:
        return [t for t in self.tasks.values() if t.status == TaskStatus.PENDING]

    @property
    def available_robots(self) -> list[Robot]:
        return [r for r in self.robots.values() if r.is_available]

    # -- serialization ------------------------------------------------------

    def to_dict(self) -> dict:
        """Lightweight JSON-serializable snapshot for the API / dashboard."""
        return {
            "rows": self.rows,
            "cols": self.cols,
            "tick": self.tick,
            "grid": [[cell.value for cell in row] for row in self.grid],
            "robots": [
                {
                    "id": r.id,
                    "row": r.position.row,
                    "col": r.position.col,
                    "status": r.status.value,
                    "battery": round(r.battery, 1),
                    "carrying_task": r.carrying_task,
                    "tasks_completed": r.tasks_completed,
                    "path": [{"row": p.row, "col": p.col} for p in r.path[r.path_index:]],
                }
                for r in self.robots.values()
            ],
            "tasks": [
                {
                    "id": t.id,
                    "pickup": {"row": t.pickup.row, "col": t.pickup.col},
                    "dropoff": {"row": t.dropoff.row, "col": t.dropoff.col},
                    "status": t.status.value,
                    "priority": t.priority,
                    "assigned_robot": t.assigned_robot,
                }
                for t in self.tasks.values()
                if t.status != TaskStatus.COMPLETED
            ],
            "stats": {
                "total_tasks": len(self.tasks),
                "completed": len(self.completed_tasks),
                "pending": len(self.pending_tasks),
                "robots_idle": sum(1 for r in self.robots.values() if r.status == RobotStatus.IDLE),
                "robots_busy": sum(1 for r in self.robots.values() if r.status != RobotStatus.IDLE),
            },
        }
