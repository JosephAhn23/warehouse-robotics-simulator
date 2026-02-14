"""
FastAPI REST API for the warehouse robotics simulator.

Provides endpoints to:
  - Query warehouse state
  - Step the simulation forward
  - Add robots and tasks
  - Run batch simulations
  - Serve the visualization dashboard
"""

from __future__ import annotations

import os
from contextlib import asynccontextmanager
from typing import Optional

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from warehouse.models import Position, Robot, Task
from warehouse.simulator import Simulator

# ---------------------------------------------------------------------------
# App state — simulator singleton
# ---------------------------------------------------------------------------

_sim: Optional[Simulator] = None


def get_sim() -> Simulator:
    global _sim
    if _sim is None:
        _sim = Simulator.create_demo()
    return _sim


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize the simulator on startup."""
    get_sim()
    yield


# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app = FastAPI(
    title="Warehouse Robotics Simulator",
    description=(
        "Multi-agent path planning and task allocation for "
        "autonomous warehouse robots."
    ),
    version="1.0.0",
    lifespan=lifespan,
)

# Serve static files (dashboard)
STATIC_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "static")
if os.path.isdir(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


# ---------------------------------------------------------------------------
# Request / response schemas
# ---------------------------------------------------------------------------

class TaskCreate(BaseModel):
    pickup_row: int
    pickup_col: int
    dropoff_row: int
    dropoff_col: int
    priority: int = Field(default=1, ge=1, le=5)


class RobotCreate(BaseModel):
    row: int
    col: int
    battery: float = Field(default=100.0, ge=0, le=100)


class ResetRequest(BaseModel):
    rows: int = 20
    cols: int = 30
    n_robots: int = 5
    n_tasks: int = 10
    obstacle_density: float = Field(default=0.15, ge=0, le=0.4)
    seed: int = 42


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/", response_class=HTMLResponse)
async def dashboard():
    """Serve the visualization dashboard."""
    index = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index):
        return FileResponse(index)
    return HTMLResponse("<h1>Warehouse Robotics Simulator API</h1><p>Dashboard not found.</p>")


@app.get("/api/state")
async def get_state():
    """Return the full warehouse state snapshot."""
    return get_sim().warehouse.to_dict()


@app.post("/api/step")
async def step_simulation(n: int = Query(default=1, ge=1, le=100)):
    """Advance the simulation by *n* ticks."""
    sim = get_sim()
    results = [sim.step() for _ in range(n)]
    return {
        "ticks_advanced": n,
        "current_tick": sim.warehouse.tick,
        "results": results,
    }


@app.post("/api/run")
async def run_simulation(ticks: int = Query(default=50, ge=1, le=500)):
    """Run a batch of ticks and return the final state + events."""
    sim = get_sim()
    results = sim.run(ticks)
    return {
        "ticks_run": ticks,
        "current_tick": sim.warehouse.tick,
        "final_stats": sim.warehouse.to_dict()["stats"],
        "events": [e for r in results for e in r["events"]],
    }


@app.post("/api/task")
async def add_task(req: TaskCreate):
    """Create a new pick-and-place task."""
    sim = get_sim()
    wh = sim.warehouse

    pickup = Position(req.pickup_row, req.pickup_col)
    dropoff = Position(req.dropoff_row, req.dropoff_col)

    if not wh.is_walkable(pickup):
        raise HTTPException(400, "Pickup position is not walkable")
    if not wh.is_walkable(dropoff):
        raise HTTPException(400, "Dropoff position is not walkable")

    task = Task(pickup=pickup, dropoff=dropoff, priority=req.priority)
    wh.add_task(task)
    return {"task_id": task.id, "status": task.status.value}


@app.post("/api/task/random")
async def add_random_task():
    """Create a random task."""
    task = get_sim().add_random_task()
    return {"task_id": task.id, "pickup": [task.pickup.row, task.pickup.col],
            "dropoff": [task.dropoff.row, task.dropoff.col]}


@app.post("/api/robot")
async def add_robot(req: RobotCreate):
    """Add a new robot to the warehouse."""
    sim = get_sim()
    pos = Position(req.row, req.col)
    if not sim.warehouse.is_walkable(pos):
        raise HTTPException(400, "Position is not walkable")

    robot = Robot(position=pos, battery=req.battery)
    sim.warehouse.add_robot(robot)
    return {"robot_id": robot.id, "position": [pos.row, pos.col]}


@app.post("/api/reset")
async def reset_simulation(req: ResetRequest):
    """Reset the simulation with new parameters."""
    global _sim
    _sim = Simulator.create_demo(
        rows=req.rows,
        cols=req.cols,
        n_robots=req.n_robots,
        n_tasks=req.n_tasks,
        obstacle_density=req.obstacle_density,
        seed=req.seed,
    )
    return {
        "message": "Simulation reset",
        "config": req.model_dump(),
        "stats": _sim.warehouse.to_dict()["stats"],
    }


@app.get("/api/robots")
async def list_robots():
    """List all robots and their current states."""
    return get_sim().warehouse.to_dict()["robots"]


@app.get("/api/tasks")
async def list_tasks():
    """List all active tasks."""
    return get_sim().warehouse.to_dict()["tasks"]


# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
