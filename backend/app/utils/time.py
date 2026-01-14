"""Time utilities for consistent UTC handling and ISO8601 serialization."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Optional


def now_utc() -> datetime:
    """Return current time as timezone-aware UTC datetime."""
    return datetime.now(timezone.utc)


def ensure_utc(dt: datetime) -> datetime:
    """
    Ensure a datetime is timezone-aware in UTC.

    - If dt is naive, assume it is UTC and attach UTC tzinfo.
    - If dt has tzinfo, convert to UTC.
    """
    if dt.tzinfo is None:
        return dt.replace(tzinfo=timezone.utc)
    return dt.astimezone(timezone.utc)


def isoformat_utc(dt: Optional[datetime] = None) -> str:
    """
    Serialize datetime as ISO8601 in UTC with 'Z' suffix.

    If dt is None, use current UTC time.
    """
    if dt is None:
        dt = now_utc()
    dt_utc = ensure_utc(dt)
    # Use 'Z' for UTC instead of '+00:00' for readability/consistency
    return dt_utc.replace(tzinfo=timezone.utc).isoformat().replace("+00:00", "Z")


def parse_iso8601(value: str) -> datetime:
    """
    Parse ISO8601 string into a timezone-aware UTC datetime.

    Raises ValueError if the string has no timezone information.
    """
    # Handle common 'Z' suffix for UTC
    if value.endswith("Z"):
        value = value[:-1] + "+00:00"

    dt = datetime.fromisoformat(value)
    if dt.tzinfo is None:
        # We require timezone information as part of the contract
        raise ValueError("Timestamp must include timezone information")

    return dt.astimezone(timezone.utc)

