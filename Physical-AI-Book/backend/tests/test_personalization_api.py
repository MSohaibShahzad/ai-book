"""
Tests for Personalization API Endpoints

Tests verify that API endpoints:
1. Return 200 on successful personalization
2. Return 400 for incomplete profile
3. Return 401 for unauthorized requests
4. Return 408 on timeout
5. Return 429 when rate limit exceeded
6. Return 500 on generation failure
7. Metrics endpoint works correctly

Feature: Chapter Personalization (005-chapter-personalization)
Task: T041
Target: >80% code coverage
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, AsyncMock
from uuid import uuid4
from datetime import datetime, timedelta


class TestPersonalizeEndpoint:
    """Tests for POST /api/personalization/personalize"""

    def test_successful_personalization(self, client, auth_headers):
        """Test 200 response on successful personalization"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.return_value = "# Personalized Chapter\n\nAdapted content."

            response = client.post(
                "/api/personalization/personalize",
                json={
                    "chapter_id": "foundations-ros2/what-is-ros2",
                    "chapter_content": "# What is ROS 2?\n\nOriginal content.",
                    "user_profile": {
                        "softwareBackground": "Intermediate",
                        "hardwareBackground": "Beginner",
                        "interestArea": "AI",
                    },
                },
                headers=auth_headers,
            )

            assert response.status_code == 200
            data = response.json()
            assert "personalized_content" in data
            assert data["personalized_content"] == "# Personalized Chapter\n\nAdapted content."
            assert "remaining_limit" in data
            assert "generation_time_ms" in data
            assert data["remaining_limit"] >= 0
            assert data["generation_time_ms"] > 0

    def test_incomplete_profile_returns_400(self, client, auth_headers):
        """Test 400 when profile is incomplete"""
        response = client.post(
            "/api/personalization/personalize",
            json={
                "chapter_id": "test-chapter",
                "chapter_content": "# Test",
                "user_profile": {
                    "softwareBackground": "Intermediate",
                    "hardwareBackground": "",  # Missing
                    "interestArea": "AI",
                },
            },
            headers=auth_headers,
        )

        assert response.status_code == 400
        data = response.json()
        assert data["detail"]["error"] == "incomplete_profile"
        assert "hardwareBackground" in data["detail"]["details"]["missing_fields"]

    def test_unauthorized_returns_401(self, client):
        """Test 401 when not authenticated"""
        response = client.post(
            "/api/personalization/personalize",
            json={
                "chapter_id": "test-chapter",
                "chapter_content": "# Test",
                "user_profile": {
                    "softwareBackground": "Beginner",
                    "hardwareBackground": "None",
                    "interestArea": "Robotics",
                },
            },
            # No auth headers
        )

        assert response.status_code == 401
        data = response.json()
        assert data["detail"]["error"] == "unauthorized"

    def test_timeout_returns_408(self, client, auth_headers):
        """Test 408 when personalization times out"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.side_effect = TimeoutError("Exceeded 30 seconds")

            response = client.post(
                "/api/personalization/personalize",
                json={
                    "chapter_id": "test-chapter",
                    "chapter_content": "# Test",
                    "user_profile": {
                        "softwareBackground": "Expert",
                        "hardwareBackground": "Advanced",
                        "interestArea": "Computer Vision",
                    },
                },
                headers=auth_headers,
            )

            assert response.status_code == 408
            data = response.json()
            assert data["detail"]["error"] == "timeout"
            assert data["detail"]["retry_allowed"] is True
            assert "remaining_limit" in data["detail"]

    def test_rate_limit_returns_429(self, client, auth_headers):
        """Test 429 when rate limit exceeded"""
        with patch("personalization.api.check_rate_limit") as mock_check:
            mock_check.return_value = (False, 0)  # No quota remaining

            with patch("personalization.api.get_quota_status") as mock_status:
                mock_status.return_value = {
                    "remaining_requests": 0,
                    "total_requests": 3,
                    "reset_at": (datetime.utcnow() + timedelta(hours=12)).isoformat(),
                    "hours_until_reset": 12.0,
                }

                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test-chapter",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Beginner",
                            "hardwareBackground": "None",
                            "interestArea": "General",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 429
                data = response.json()
                assert data["detail"]["error"] == "rate_limit_exceeded"
                assert data["detail"]["remaining_limit"] == 0
                assert "reset_at" in data["detail"]
                assert "retry_after_seconds" in data["detail"]

    def test_generation_failure_returns_500(self, client, auth_headers):
        """Test 500 when generation fails"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.side_effect = Exception("OpenAI API error")

            response = client.post(
                "/api/personalization/personalize",
                json={
                    "chapter_id": "test-chapter",
                    "chapter_content": "# Test",
                    "user_profile": {
                        "softwareBackground": "Intermediate",
                        "hardwareBackground": "Beginner",
                        "interestArea": "Motion Control",
                    },
                },
                headers=auth_headers,
            )

            assert response.status_code == 500
            data = response.json()
            assert data["detail"]["error"] == "generation_failed"
            assert data["detail"]["retry_allowed"] is True

    def test_decrements_quota_on_success(self, client, auth_headers):
        """Test quota decremented after successful generation"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.return_value = "# Personalized"

            with patch("personalization.api.increment_quota") as mock_increment:
                mock_increment.return_value = 2  # 2 remaining after this request

                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test-chapter",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Beginner",
                            "hardwareBackground": "None",
                            "interestArea": "AI",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 200
                data = response.json()
                assert data["remaining_limit"] == 2
                mock_increment.assert_called_once()

    def test_decrements_quota_on_timeout(self, client, auth_headers):
        """Test quota still decremented even on timeout"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.side_effect = TimeoutError("Too slow")

            with patch("personalization.api.increment_quota") as mock_increment:
                mock_increment.return_value = 1

                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test-chapter",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Intermediate",
                            "hardwareBackground": "Beginner",
                            "interestArea": "Robotics",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 408
                # Quota should still be decremented
                mock_increment.assert_called_once()

    def test_validates_request_body(self, client, auth_headers):
        """Test request body validation"""
        # Missing chapter_id
        response = client.post(
            "/api/personalization/personalize",
            json={
                "chapter_content": "# Test",
                "user_profile": {
                    "softwareBackground": "Beginner",
                    "hardwareBackground": "None",
                    "interestArea": "AI",
                },
            },
            headers=auth_headers,
        )

        assert response.status_code == 422  # Validation error


class TestQuotaEndpoint:
    """Tests for GET /api/personalization/quota"""

    def test_returns_quota_status(self, client, auth_headers):
        """Test 200 response with quota status"""
        with patch("personalization.api.get_quota_status") as mock_status:
            mock_status.return_value = {
                "remaining_requests": 3,
                "total_requests": 3,
                "reset_at": "2025-12-25T10:00:00Z",
                "hours_until_reset": 24.0,
            }

            response = client.get("/api/personalization/quota", headers=auth_headers)

            assert response.status_code == 200
            data = response.json()
            assert data["remaining_requests"] == 3
            assert data["total_requests"] == 3
            assert data["reset_at"] == "2025-12-25T10:00:00Z"
            assert data["hours_until_reset"] == 24.0

    def test_unauthorized_returns_401(self, client):
        """Test 401 when not authenticated"""
        response = client.get("/api/personalization/quota")

        assert response.status_code == 401

    def test_returns_exhausted_quota(self, client, auth_headers):
        """Test response when quota exhausted"""
        with patch("personalization.api.get_quota_status") as mock_status:
            mock_status.return_value = {
                "remaining_requests": 0,
                "total_requests": 3,
                "reset_at": "2025-12-25T10:00:00Z",
                "hours_until_reset": 12.5,
            }

            response = client.get("/api/personalization/quota", headers=auth_headers)

            assert response.status_code == 200
            data = response.json()
            assert data["remaining_requests"] == 0


class TestCacheInvalidationEndpoint:
    """Tests for DELETE /api/personalization/cache"""

    def test_returns_204(self, client, auth_headers):
        """Test 204 No Content on success"""
        response = client.delete("/api/personalization/cache", headers=auth_headers)

        assert response.status_code == 204
        assert response.content == b""

    def test_unauthorized_returns_401(self, client):
        """Test 401 when not authenticated"""
        response = client.delete("/api/personalization/cache")

        assert response.status_code == 401


class TestMetricsEndpoint:
    """Tests for GET /api/personalization/metrics"""

    def test_returns_metrics(self, client):
        """Test metrics endpoint returns data (no auth required)"""
        with patch("personalization.api.metrics") as mock_metrics:
            mock_metrics.to_dict.return_value = {
                "total_requests": 100,
                "successful_requests": 90,
                "failed_requests": 5,
                "timeout_requests": 5,
                "avg_duration_ms": 18500.0,
                "success_rate_percent": 90.0,
                "last_updated": "2025-12-24T10:00:00Z",
            }

            response = client.get("/api/personalization/metrics")

            assert response.status_code == 200
            data = response.json()
            assert data["total_requests"] == 100
            assert data["successful_requests"] == 90
            assert data["failed_requests"] == 5
            assert data["timeout_requests"] == 5
            assert data["avg_duration_ms"] == 18500.0
            assert data["success_rate_percent"] == 90.0

    def test_no_authentication_required(self, client):
        """Test metrics accessible without authentication"""
        response = client.get("/api/personalization/metrics")

        # Should not return 401
        assert response.status_code == 200

    def test_handles_zero_requests(self, client):
        """Test metrics when no requests made yet"""
        with patch("personalization.api.metrics") as mock_metrics:
            mock_metrics.to_dict.return_value = {
                "total_requests": 0,
                "successful_requests": 0,
                "failed_requests": 0,
                "timeout_requests": 0,
                "avg_duration_ms": 0.0,
                "success_rate_percent": 0.0,
                "last_updated": None,
            }

            response = client.get("/api/personalization/metrics")

            assert response.status_code == 200
            data = response.json()
            assert data["total_requests"] == 0
            assert data["success_rate_percent"] == 0.0


class TestMetricsTracking:
    """Tests for metrics tracking during requests"""

    def test_records_success_metrics(self, client, auth_headers):
        """Test successful request updates metrics"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.return_value = "# Personalized"

            with patch("personalization.api.metrics") as mock_metrics:
                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Intermediate",
                            "hardwareBackground": "Beginner",
                            "interestArea": "AI",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 200
                # Metrics should record success
                mock_metrics.record_success.assert_called_once()
                # Duration should be positive
                call_args = mock_metrics.record_success.call_args[0]
                assert call_args[0] > 0  # duration_ms > 0

    def test_records_timeout_metrics(self, client, auth_headers):
        """Test timeout updates metrics"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.side_effect = TimeoutError()

            with patch("personalization.api.metrics") as mock_metrics:
                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Expert",
                            "hardwareBackground": "Advanced",
                            "interestArea": "Robotics",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 408
                # Metrics should record timeout
                mock_metrics.record_failure.assert_called_once()
                call_args = mock_metrics.record_failure.call_args
                assert call_args[1]["is_timeout"] is True

    def test_records_failure_metrics(self, client, auth_headers):
        """Test failure updates metrics"""
        with patch("personalization.api.personalize_chapter_async") as mock_personalize:
            mock_personalize.side_effect = Exception("Generation error")

            with patch("personalization.api.metrics") as mock_metrics:
                response = client.post(
                    "/api/personalization/personalize",
                    json={
                        "chapter_id": "test",
                        "chapter_content": "# Test",
                        "user_profile": {
                            "softwareBackground": "Beginner",
                            "hardwareBackground": "None",
                            "interestArea": "General",
                        },
                    },
                    headers=auth_headers,
                )

                assert response.status_code == 500
                # Metrics should record failure
                mock_metrics.record_failure.assert_called_once()
                call_args = mock_metrics.record_failure.call_args
                assert call_args[1]["is_timeout"] is False


# Fixtures

@pytest.fixture
def client():
    """Create FastAPI test client"""
    from personalization.api import router
    from fastapi import FastAPI

    app = FastAPI()
    app.include_router(router)

    from fastapi.testclient import TestClient
    return TestClient(app)


@pytest.fixture
def auth_headers():
    """Create authentication headers"""
    user_id = str(uuid4())
    session_token = "test-session-token"

    # Mock Better-Auth session
    headers = {"Cookie": f"better-auth-session={session_token}"}

    with patch("personalization.api.get_current_user_id") as mock_get_user:
        mock_get_user.return_value = user_id
        yield headers


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--cov=personalization", "--cov-report=term-missing"])
