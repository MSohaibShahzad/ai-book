-- Migration: Create translation_cache table
-- Feature: 004-urdu-translation
-- Date: 2025-12-20
-- Description: Stores cached translations to minimize OpenAI API costs

CREATE TABLE IF NOT EXISTS translation_cache (
    id SERIAL PRIMARY KEY,
    chapter_slug VARCHAR(255) NOT NULL,
    language VARCHAR(10) NOT NULL,
    content_hash VARCHAR(64) NOT NULL,
    translated_content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    accessed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP NOT NULL,
    UNIQUE(chapter_slug, language, content_hash)
);

-- Index for cache lookup (chapter + language + content version)
CREATE INDEX idx_translation_lookup
ON translation_cache(chapter_slug, language, content_hash);

-- Index for cleanup of expired entries
CREATE INDEX idx_translation_expiry
ON translation_cache(expires_at);

-- Comments for documentation
COMMENT ON TABLE translation_cache IS 'Cached translations with version-based invalidation via content_hash';
COMMENT ON COLUMN translation_cache.content_hash IS 'SHA-256 hash of source markdown content for version tracking';
COMMENT ON COLUMN translation_cache.expires_at IS 'TTL-based expiration (90 days default, 30-180 days based on chapter type)';
