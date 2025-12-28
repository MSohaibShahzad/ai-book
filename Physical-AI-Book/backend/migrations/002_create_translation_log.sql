-- Migration: Create translation_log table
-- Feature: 004-urdu-translation
-- Date: 2025-12-20
-- Description: Tracks translation usage for analytics

CREATE TABLE IF NOT EXISTS translation_log (
    id SERIAL PRIMARY KEY,
    user_id TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    chapter_slug VARCHAR(255) NOT NULL,
    action VARCHAR(50) NOT NULL,
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Index for querying by user
CREATE INDEX idx_translation_log_user
ON translation_log(user_id);

-- Index for querying recent activity (descending timestamp)
CREATE INDEX idx_translation_log_timestamp
ON translation_log(timestamp DESC);

-- Index for chapter-based analytics
CREATE INDEX idx_translation_log_chapter
ON translation_log(chapter_slug);

-- Comments for documentation
COMMENT ON TABLE translation_log IS 'Analytics tracking for translation feature usage';
COMMENT ON COLUMN translation_log.action IS 'Action type: translate_requested, toggle_to_english, toggle_to_urdu, translation_failed';
