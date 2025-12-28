"""
Personalization Database Models

SQLAlchemy models for personalization_quota table.
"""

from sqlalchemy import Column, Integer, String, CheckConstraint, ForeignKey, TIMESTAMP
from sqlalchemy.sql import func
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()


class PersonalizationQuota(Base):
    """
    Tracks personalization requests per user with rolling 24-hour window.

    Enforces 3 requests per user per day limit.
    """

    __tablename__ = "personalization_quota"

    user_id = Column(String(255), primary_key=True)  # Better-Auth uses string IDs, not UUIDs
    request_count = Column(
        Integer,
        nullable=False,
        default=0,
        # Enforced by CHECK constraint in migration
    )
    first_request_timestamp = Column(TIMESTAMP(timezone=True), nullable=False)
    reset_at = Column(TIMESTAMP(timezone=True), nullable=False)
    created_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(TIMESTAMP(timezone=True), nullable=False, server_default=func.now(), onupdate=func.now())

    __table_args__ = (
        CheckConstraint("request_count >= 0 AND request_count <= 3", name="chk_request_count"),
    )

    def __repr__(self):
        return f"<PersonalizationQuota(user_id={self.user_id}, request_count={self.request_count}, reset_at={self.reset_at})>"
