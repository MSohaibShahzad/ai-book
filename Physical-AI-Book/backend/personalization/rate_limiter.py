"""
Rate Limiting Logic

Implements rolling 24-hour window rate limiting (3 requests per user per day).
Uses PostgreSQL personalization_quota table for persistent storage.
"""

from typing import Tuple, Optional
from datetime import datetime, timedelta, timezone
from sqlalchemy.orm import Session
from sqlalchemy import create_engine
from sqlalchemy.exc import IntegrityError
import os
import logging

from .models import PersonalizationQuota

logger = logging.getLogger(__name__)

# Database connection
DATABASE_URL = os.getenv("DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable not set")

engine = create_engine(DATABASE_URL)

# Configuration
PERSONALIZATION_DAILY_LIMIT = int(os.getenv("PERSONALIZATION_DAILY_LIMIT", "3"))


def get_db_session() -> Session:
    """Create new database session."""
    from sqlalchemy.orm import sessionmaker
    SessionLocal = sessionmaker(bind=engine)
    return SessionLocal()


def check_rate_limit(user_id: str, db: Optional[Session] = None) -> Tuple[bool, int]:
    """
    Check if user has remaining personalization requests.

    Implements rolling 24-hour window:
    - First request: Creates quota entry with reset_at = now + 24 hours
    - Subsequent requests: Increments request_count if within window
    - Expired window: Resets quota automatically

    Args:
        user_id: User UUID
        db: Optional database session (creates new session if None)

    Returns:
        Tuple of (is_allowed, remaining_requests)
        - is_allowed: True if user can make request, False if limit exceeded
        - remaining_requests: Number of requests remaining (0-3)
    """
    close_session = False
    if db is None:
        db = get_db_session()
        close_session = True

    try:
        # Query existing quota
        quota = db.query(PersonalizationQuota).filter_by(user_id=user_id).first()

        now = datetime.now(timezone.utc)

        if not quota:
            # First request - create quota entry
            logger.info(f"Creating quota entry for user {user_id}")
            quota = PersonalizationQuota(
                user_id=user_id,
                request_count=0,  # Will be incremented by increment_quota()
                first_request_timestamp=now,
                reset_at=now + timedelta(hours=24)
            )
            db.add(quota)
            db.commit()
            db.refresh(quota)

            # User can make request, has 3 remaining (will be 2 after increment)
            return True, PERSONALIZATION_DAILY_LIMIT

        # Check if window expired
        if now > quota.reset_at:
            # Reset quota
            logger.info(f"Resetting expired quota for user {user_id}")
            quota.request_count = 0
            quota.first_request_timestamp = now
            quota.reset_at = now + timedelta(hours=24)
            quota.updated_at = now
            db.commit()

            return True, PERSONALIZATION_DAILY_LIMIT

        # Check if limit exceeded
        if quota.request_count >= PERSONALIZATION_DAILY_LIMIT:
            remaining = 0
            logger.warning(f"Rate limit exceeded for user {user_id}")
            return False, remaining

        # User can make request
        remaining = PERSONALIZATION_DAILY_LIMIT - quota.request_count
        logger.info(f"User {user_id} has {remaining} requests remaining")
        return True, remaining

    except Exception as e:
        logger.error(f"Error checking rate limit for user {user_id}: {e}")
        db.rollback()
        # Fail open - allow request on error
        return True, PERSONALIZATION_DAILY_LIMIT

    finally:
        if close_session:
            db.close()


def increment_quota(user_id: str, db: Optional[Session] = None) -> int:
    """
    Increment user's request count atomically.

    Should be called AFTER successful personalization to decrement remaining quota.

    Args:
        user_id: User UUID
        db: Optional database session (creates new session if None)

    Returns:
        Remaining requests after increment (0-2)

    Raises:
        ValueError: If quota doesn't exist (should call check_rate_limit first)
    """
    close_session = False
    if db is None:
        db = get_db_session()
        close_session = True

    try:
        # Query existing quota
        quota = db.query(PersonalizationQuota).filter_by(user_id=user_id).first()

        if not quota:
            raise ValueError(f"Quota not found for user {user_id}. Call check_rate_limit first.")

        # Increment count
        quota.request_count += 1
        quota.updated_at = datetime.now(timezone.utc)
        db.commit()
        db.refresh(quota)

        remaining = PERSONALIZATION_DAILY_LIMIT - quota.request_count
        logger.info(f"Incremented quota for user {user_id}. Remaining: {remaining}")

        return remaining

    except Exception as e:
        logger.error(f"Error incrementing quota for user {user_id}: {e}")
        db.rollback()
        raise

    finally:
        if close_session:
            db.close()


def get_quota_status(user_id: str, db: Optional[Session] = None) -> dict:
    """
    Get detailed quota status for user.

    Args:
        user_id: User UUID
        db: Optional database session

    Returns:
        Dictionary with:
        - remaining_requests: Number of remaining requests (0-3)
        - total_requests: Total allowed requests per day (3)
        - reset_at: ISO timestamp when quota resets
        - hours_until_reset: Hours until quota resets (float)
    """
    close_session = False
    if db is None:
        db = get_db_session()
        close_session = True

    try:
        quota = db.query(PersonalizationQuota).filter_by(user_id=user_id).first()

        now = datetime.now(timezone.utc)

        if not quota or now > quota.reset_at:
            # No quota or expired - user has full quota
            return {
                "remaining_requests": PERSONALIZATION_DAILY_LIMIT,
                "total_requests": PERSONALIZATION_DAILY_LIMIT,
                "reset_at": (now + timedelta(hours=24)).isoformat().replace('+00:00', 'Z'),
                "hours_until_reset": 24.0
            }

        remaining = PERSONALIZATION_DAILY_LIMIT - quota.request_count
        hours_until_reset = (quota.reset_at - now).total_seconds() / 3600

        return {
            "remaining_requests": max(0, remaining),
            "total_requests": PERSONALIZATION_DAILY_LIMIT,
            "reset_at": quota.reset_at.isoformat().replace('+00:00', 'Z'),
            "hours_until_reset": max(0, hours_until_reset)
        }

    except Exception as e:
        logger.error(f"Error getting quota status for user {user_id}: {e}")
        # Return full quota on error
        return {
            "remaining_requests": PERSONALIZATION_DAILY_LIMIT,
            "total_requests": PERSONALIZATION_DAILY_LIMIT,
            "reset_at": (datetime.now(timezone.utc) + timedelta(hours=24)).isoformat().replace('+00:00', 'Z'),
            "hours_until_reset": 24.0
        }

    finally:
        if close_session:
            db.close()


def reset_quota_if_expired(user_id: str, db: Optional[Session] = None) -> bool:
    """
    Reset quota if 24 hours have elapsed since first request.

    This is called lazily by check_rate_limit, but can also be called explicitly.

    Args:
        user_id: User UUID
        db: Optional database session

    Returns:
        True if quota was reset, False otherwise
    """
    close_session = False
    if db is None:
        db = get_db_session()
        close_session = True

    try:
        quota = db.query(PersonalizationQuota).filter_by(user_id=user_id).first()

        if not quota:
            return False

        now = datetime.now(timezone.utc)

        if now > quota.reset_at:
            logger.info(f"Manually resetting quota for user {user_id}")
            quota.request_count = 0
            quota.first_request_timestamp = now
            quota.reset_at = now + timedelta(hours=24)
            quota.updated_at = now
            db.commit()
            return True

        return False

    except Exception as e:
        logger.error(f"Error resetting quota for user {user_id}: {e}")
        db.rollback()
        return False

    finally:
        if close_session:
            db.close()
