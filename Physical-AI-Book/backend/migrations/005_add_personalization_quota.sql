-- Migration: 005_add_personalization_quota
-- Description: Create personalization_quota table for rate limiting (3 requests/day)
-- Feature: Chapter Personalization (005-chapter-personalization)
-- Date: 2025-12-23

-- Create personalization_quota table
CREATE TABLE IF NOT EXISTS personalization_quota (
    user_id UUID PRIMARY KEY,
    request_count INTEGER NOT NULL DEFAULT 0,
    first_request_timestamp TIMESTAMPTZ NOT NULL,
    reset_at TIMESTAMPTZ NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Foreign key to users table (assumes users.id exists from feature 003-auth)
    CONSTRAINT fk_user FOREIGN KEY (user_id)
        REFERENCES users(id)
        ON DELETE CASCADE,

    -- Enforce 0-3 request count constraint
    CONSTRAINT chk_request_count
        CHECK (request_count >= 0 AND request_count <= 3)
);

-- Create indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_personalization_quota_reset
    ON personalization_quota(reset_at);

CREATE INDEX IF NOT EXISTS idx_personalization_quota_user
    ON personalization_quota(user_id);

-- Add comment for documentation
COMMENT ON TABLE personalization_quota IS
    'Tracks personalization requests per user with rolling 24-hour window (3 requests/day limit)';

COMMENT ON COLUMN personalization_quota.user_id IS
    'User identifier (references users.id from Better-Auth)';

COMMENT ON COLUMN personalization_quota.request_count IS
    'Number of personalization requests made in current 24-hour window (0-3)';

COMMENT ON COLUMN personalization_quota.first_request_timestamp IS
    'Timestamp of first request in current window (used to calculate 24-hour expiration)';

COMMENT ON COLUMN personalization_quota.reset_at IS
    'Timestamp when quota resets (first_request_timestamp + 24 hours)';
