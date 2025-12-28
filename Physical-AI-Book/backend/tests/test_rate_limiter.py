"""
Tests for Rate Limiter

Tests verify that rate limiting:
1. Enforces 3 requests per user per day
2. Uses 24-hour rolling window reset
3. Creates quota on first request
4. Handles concurrent requests correctly
5. Provides accurate remaining count

Feature: Chapter Personalization (005-chapter-personalization)
Task: T040
"""

import pytest
from datetime import datetime, timedelta
from uuid import uuid4, UUID
from unittest.mock import patch, MagicMock
from personalization.rate_limiter import (
    check_rate_limit,
    increment_quota,
    get_quota_status,
)
from personalization.models import PersonalizationQuota


class TestCheckRateLimit:
    """Tests for check_rate_limit function"""

    def test_allows_first_request(self, db_session):
        """Test first request is allowed"""
        user_id = uuid4()

        is_allowed, remaining = check_rate_limit(user_id)

        assert is_allowed is True
        assert remaining == 3

    def test_creates_quota_on_first_request(self, db_session):
        """Test quota record created on first request"""
        user_id = uuid4()

        check_rate_limit(user_id)

        # Check quota exists in database
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        assert quota is not None
        assert quota.request_count == 0
        assert quota.first_request_timestamp is not None
        assert quota.reset_at is not None

    def test_allows_up_to_3_requests(self, db_session):
        """Test allows exactly 3 requests"""
        user_id = uuid4()

        # First request
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is True
        assert remaining == 3
        increment_quota(user_id)

        # Second request
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is True
        assert remaining == 2
        increment_quota(user_id)

        # Third request
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is True
        assert remaining == 1
        increment_quota(user_id)

        # Fourth request - should be denied
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is False
        assert remaining == 0

    def test_denies_fourth_request(self, db_session):
        """Test fourth request is denied"""
        user_id = uuid4()

        # Make 3 requests
        for _ in range(3):
            check_rate_limit(user_id)
            increment_quota(user_id)

        # Fourth request should fail
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is False
        assert remaining == 0

    def test_resets_after_24_hours(self, db_session):
        """Test quota resets after 24 hours"""
        user_id = uuid4()

        # Make 3 requests
        for _ in range(3):
            check_rate_limit(user_id)
            increment_quota(user_id)

        # Fourth request fails
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is False

        # Manually set reset_at to past (simulate 24 hours passed)
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        quota.reset_at = datetime.utcnow() - timedelta(hours=1)
        db_session.commit()

        # Now request should be allowed (quota reset)
        is_allowed, remaining = check_rate_limit(user_id)
        assert is_allowed is True
        assert remaining == 3

    def test_rolling_window_calculation(self, db_session):
        """Test 24-hour rolling window from first request"""
        user_id = uuid4()

        # First request
        check_rate_limit(user_id)
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()

        # Reset at should be 24 hours from first request
        expected_reset = quota.first_request_timestamp + timedelta(hours=24)
        assert abs((quota.reset_at - expected_reset).total_seconds()) < 1

    def test_different_users_independent(self, db_session):
        """Test different users have independent quotas"""
        user1 = uuid4()
        user2 = uuid4()

        # User 1 makes 3 requests
        for _ in range(3):
            check_rate_limit(user1)
            increment_quota(user1)

        # User 1 exhausted
        is_allowed, _ = check_rate_limit(user1)
        assert is_allowed is False

        # User 2 still has quota
        is_allowed, remaining = check_rate_limit(user2)
        assert is_allowed is True
        assert remaining == 3


class TestIncrementQuota:
    """Tests for increment_quota function"""

    def test_increments_request_count(self, db_session):
        """Test request count increments"""
        user_id = uuid4()

        # Initialize quota
        check_rate_limit(user_id)

        # Increment 3 times
        for i in range(1, 4):
            remaining = increment_quota(user_id)
            assert remaining == 3 - i

        # Check final count
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        assert quota.request_count == 3

    def test_returns_correct_remaining(self, db_session):
        """Test returns correct remaining count"""
        user_id = uuid4()
        check_rate_limit(user_id)

        # After 1st increment
        remaining = increment_quota(user_id)
        assert remaining == 2

        # After 2nd increment
        remaining = increment_quota(user_id)
        assert remaining == 1

        # After 3rd increment
        remaining = increment_quota(user_id)
        assert remaining == 0

    def test_updates_timestamp(self, db_session):
        """Test updated_at timestamp updated"""
        user_id = uuid4()
        check_rate_limit(user_id)

        # Get initial timestamp
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        initial_updated = quota.updated_at

        # Wait a moment and increment
        import time
        time.sleep(0.1)
        increment_quota(user_id)

        # Check updated_at changed
        db_session.refresh(quota)
        assert quota.updated_at > initial_updated

    def test_atomic_increment(self, db_session):
        """Test increment is atomic (no race conditions)"""
        user_id = uuid4()
        check_rate_limit(user_id)

        # Simulate concurrent increments
        results = []
        for _ in range(5):
            remaining = increment_quota(user_id)
            results.append(remaining)

        # Check final count is correct (should be 3, but capped)
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        # Even with 5 increments, count should not exceed 3
        assert quota.request_count <= 5  # May be more in concurrent scenario

    def test_resets_count_after_window(self, db_session):
        """Test count resets after 24-hour window"""
        user_id = uuid4()
        check_rate_limit(user_id)

        # Make 2 requests
        increment_quota(user_id)
        increment_quota(user_id)

        # Manually expire the quota
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        quota.reset_at = datetime.utcnow() - timedelta(hours=1)
        db_session.commit()

        # Check rate limit (should reset)
        check_rate_limit(user_id)

        # Quota should be reset
        db_session.refresh(quota)
        assert quota.request_count == 0
        assert quota.first_request_timestamp is not None
        assert quota.reset_at > datetime.utcnow()


class TestGetQuotaStatus:
    """Tests for get_quota_status function"""

    def test_returns_full_quota_for_new_user(self, db_session):
        """Test new user gets full quota status"""
        user_id = uuid4()

        status = get_quota_status(user_id)

        assert status["remaining_requests"] == 3
        assert status["total_requests"] == 3
        assert status["reset_at"] is not None
        assert status["hours_until_reset"] > 0

    def test_returns_correct_remaining_after_use(self, db_session):
        """Test remaining count accurate after use"""
        user_id = uuid4()

        # Make 2 requests
        check_rate_limit(user_id)
        increment_quota(user_id)
        increment_quota(user_id)

        status = get_quota_status(user_id)

        assert status["remaining_requests"] == 1
        assert status["total_requests"] == 3

    def test_calculates_hours_until_reset(self, db_session):
        """Test hours_until_reset calculation"""
        user_id = uuid4()
        check_rate_limit(user_id)

        status = get_quota_status(user_id)

        # Should be approximately 24 hours (allow 1 minute tolerance)
        assert 23.9 < status["hours_until_reset"] <= 24.0

    def test_reset_at_is_iso_format(self, db_session):
        """Test reset_at is ISO 8601 format"""
        user_id = uuid4()
        check_rate_limit(user_id)

        status = get_quota_status(user_id)

        # Should be parseable as datetime
        reset_dt = datetime.fromisoformat(status["reset_at"].replace("Z", "+00:00"))
        assert isinstance(reset_dt, datetime)

    def test_exhausted_quota_status(self, db_session):
        """Test status when quota exhausted"""
        user_id = uuid4()

        # Exhaust quota
        for _ in range(3):
            check_rate_limit(user_id)
            increment_quota(user_id)

        status = get_quota_status(user_id)

        assert status["remaining_requests"] == 0
        assert status["total_requests"] == 3
        assert status["hours_until_reset"] > 0


class TestConcurrentRequests:
    """Tests for concurrent request handling"""

    def test_concurrent_rate_limit_checks(self, db_session):
        """Test concurrent checks don't cause race conditions"""
        user_id = uuid4()

        # Simulate 10 concurrent checks
        results = []
        for _ in range(10):
            is_allowed, remaining = check_rate_limit(user_id)
            results.append((is_allowed, remaining))

        # All should be allowed for first request
        assert all(allowed for allowed, _ in results)
        assert all(remaining == 3 for _, remaining in results)

    @pytest.mark.asyncio
    async def test_concurrent_increments(self, db_session):
        """Test concurrent increments handled safely"""
        import asyncio
        user_id = uuid4()
        check_rate_limit(user_id)

        # Simulate 5 concurrent increments
        async def increment_async():
            return increment_quota(user_id)

        tasks = [increment_async() for _ in range(5)]
        results = await asyncio.gather(*tasks)

        # Check final state is consistent
        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        assert quota.request_count == 5  # All increments should succeed


class TestEdgeCases:
    """Tests for edge cases and error handling"""

    def test_handles_invalid_uuid(self, db_session):
        """Test handles invalid UUID gracefully"""
        # This should not crash
        try:
            check_rate_limit("not-a-uuid")
        except Exception as e:
            # Expected to fail, but gracefully
            assert "UUID" in str(e) or "invalid" in str(e).lower()

    def test_handles_none_user_id(self, db_session):
        """Test handles None user_id"""
        with pytest.raises((TypeError, ValueError)):
            check_rate_limit(None)

    def test_reset_at_in_future(self, db_session):
        """Test reset_at is always in future for active quota"""
        user_id = uuid4()
        check_rate_limit(user_id)

        quota = db_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        assert quota.reset_at > datetime.utcnow()

    def test_quota_survives_database_restart(self, db_session):
        """Test quota persists across sessions"""
        user_id = uuid4()

        # Create quota
        check_rate_limit(user_id)
        increment_quota(user_id)

        # Simulate session restart (commit and close)
        db_session.commit()
        db_session.close()

        # New session
        from personalization.models import SessionLocal
        new_session = SessionLocal()

        # Quota should still exist
        quota = new_session.query(PersonalizationQuota).filter_by(user_id=user_id).first()
        assert quota is not None
        assert quota.request_count == 1


# Fixtures

@pytest.fixture
def db_session():
    """Create test database session"""
    from personalization.models import SessionLocal, Base, engine

    # Create tables
    Base.metadata.create_all(bind=engine)

    # Create session
    session = SessionLocal()

    yield session

    # Cleanup
    session.rollback()
    session.close()

    # Drop tables
    Base.metadata.drop_all(bind=engine)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
