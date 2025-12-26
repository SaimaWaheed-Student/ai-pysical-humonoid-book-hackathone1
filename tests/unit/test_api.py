import pytest
from fastapi.testclient import TestClient
from src.main import app


@pytest.fixture
def client():
    return TestClient(app)


def test_health_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["version"] == "v1"


def test_api_routes_available():
    """Test that the API routes are properly included"""
    # This just tests that the routes are defined without error
    assert len(app.routes) > 1  # Should have multiple routes