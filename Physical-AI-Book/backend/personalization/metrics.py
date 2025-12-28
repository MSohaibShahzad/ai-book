"""
Observability and Metrics Tracking

Tracks personalization request metrics for monitoring and alerting.
Exposes metrics via GET /metrics endpoint for dashboard integration.
"""

from datetime import datetime
from typing import Dict


class PersonalizationMetrics:
    """
    In-memory metrics tracker for personalization requests.

    Tracks:
    - total_requests: Total personalization requests
    - successful_requests: Successfully completed requests
    - failed_requests: Failed requests (timeouts, errors)
    - timeout_requests: Requests that exceeded 30-second timeout
    - total_duration_ms: Cumulative generation time
    """

    def __init__(self):
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.timeout_requests = 0
        self.total_duration_ms = 0.0
        self.last_updated = datetime.utcnow()

    @property
    def avg_duration_ms(self) -> float:
        """Calculate average generation time."""
        if self.total_requests == 0:
            return 0.0
        return self.total_duration_ms / self.total_requests

    @property
    def success_rate_percent(self) -> float:
        """Calculate success rate percentage."""
        if self.total_requests == 0:
            return 0.0
        return (self.successful_requests / self.total_requests) * 100

    def record_success(self, duration_ms: float):
        """Record a successful personalization request."""
        self.total_requests += 1
        self.successful_requests += 1
        self.total_duration_ms += duration_ms
        self.last_updated = datetime.utcnow()

    def record_failure(self, duration_ms: float, is_timeout: bool = False):
        """Record a failed personalization request."""
        self.total_requests += 1
        self.failed_requests += 1
        if is_timeout:
            self.timeout_requests += 1
        self.total_duration_ms += duration_ms
        self.last_updated = datetime.utcnow()

    def to_dict(self) -> Dict:
        """Export metrics as dictionary."""
        return {
            "total_requests": self.total_requests,
            "successful_requests": self.successful_requests,
            "failed_requests": self.failed_requests,
            "timeout_requests": self.timeout_requests,
            "avg_duration_ms": self.avg_duration_ms,
            "success_rate_percent": self.success_rate_percent,
            "last_updated": self.last_updated.isoformat(),
        }


# Global metrics instance
metrics = PersonalizationMetrics()
