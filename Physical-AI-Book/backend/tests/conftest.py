"""
Pytest configuration and fixtures for personalization tests

Feature: 005-chapter-personalization
"""

import os
import pytest
from unittest.mock import MagicMock, AsyncMock
from uuid import uuid4

# Set dummy API key for testing (prevents import errors)
os.environ["OPENAI_API_KEY"] = "sk-test-dummy-key-for-testing"


@pytest.fixture
def test_user_id():
    """Generate test user ID"""
    return uuid4()


@pytest.fixture
def sample_chapter_content():
    """Sample chapter markdown for testing"""
    return """---
title: Test Chapter
id: test-chapter
---

# Test Chapter

Sample content with code:

```python
def hello():
    print("world")
```

Math: $E = mc^2$

Display math:
$$
\\int_0^\\infty e^{-x} dx = 1
$$

![Image](test.png)
"""


@pytest.fixture
def sample_user_profile():
    """Sample user profile for testing"""
    return {
        "softwareBackground": "Intermediate",
        "hardwareBackground": "Beginner",
        "interestArea": "AI",
    }


@pytest.fixture
def mock_openai_agent():
    """Mock OpenAI agent for testing"""
    agent = MagicMock()
    agent.run = AsyncMock(return_value="# Personalized content")
    return agent
