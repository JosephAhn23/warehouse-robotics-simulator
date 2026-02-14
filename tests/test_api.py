"""Integration tests for the FastAPI endpoints."""

import pytest
import pytest_asyncio
from httpx import ASGITransport, AsyncClient

from api.main import app


@pytest_asyncio.fixture
async def client():
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as ac:
        # Reset before each test
        await ac.post("/api/reset", json={"rows": 10, "cols": 10, "n_robots": 2, "n_tasks": 3})
        yield ac


@pytest.mark.asyncio
async def test_get_state(client):
    resp = await client.get("/api/state")
    assert resp.status_code == 200
    data = resp.json()
    assert data["rows"] == 10
    assert data["cols"] == 10
    assert len(data["robots"]) == 2


@pytest.mark.asyncio
async def test_step(client):
    resp = await client.post("/api/step?n=5")
    assert resp.status_code == 200
    data = resp.json()
    assert data["ticks_advanced"] == 5
    assert data["current_tick"] == 5


@pytest.mark.asyncio
async def test_add_random_task(client):
    resp = await client.post("/api/task/random")
    assert resp.status_code == 200
    data = resp.json()
    assert "task_id" in data


@pytest.mark.asyncio
async def test_add_robot(client):
    resp = await client.post("/api/robot", json={"row": 0, "col": 0, "battery": 80.0})
    assert resp.status_code == 200
    data = resp.json()
    assert "robot_id" in data


@pytest.mark.asyncio
async def test_reset(client):
    # Step forward first
    await client.post("/api/step?n=10")
    # Reset
    resp = await client.post("/api/reset", json={"rows": 8, "cols": 8, "n_robots": 1, "n_tasks": 1})
    assert resp.status_code == 200
    # Verify reset
    state = (await client.get("/api/state")).json()
    assert state["rows"] == 8
    assert state["tick"] == 0


@pytest.mark.asyncio
async def test_list_robots(client):
    resp = await client.get("/api/robots")
    assert resp.status_code == 200
    robots = resp.json()
    assert len(robots) == 2


@pytest.mark.asyncio
async def test_invalid_task_position(client):
    """Adding a task on an obstacle should fail."""
    # We rely on the small grid potentially having obstacles; test the validation logic
    resp = await client.post("/api/task", json={
        "pickup_row": -1, "pickup_col": -1,
        "dropoff_row": 0, "dropoff_col": 0,
    })
    assert resp.status_code == 400
