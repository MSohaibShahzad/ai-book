#!/bin/bash
# Script to sync textbook docs into backend for Railway deployment

echo "Syncing textbook docs into backend/textbook/docs..."

# Create textbook directory in backend if it doesn't exist
mkdir -p textbook/docs

# Copy all markdown files from ../textbook/docs to backend/textbook/docs
rsync -av --delete ../textbook/docs/ ./textbook/docs/

echo "✓ Textbook docs synced successfully!"
echo "Files copied:"
find textbook/docs -name "*.md" -o -name "*.mdx" | wc -l
