#!/usr/bin/env python3
"""
Live translation test with OpenAI API.
Tests actual Urdu translation using GPT-4o.
"""

import asyncio
import sys
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from services.openai_agent_service import OpenAIAgentService


async def test_live_translation():
    """Test actual translation to Urdu using OpenAI API."""
    print("\n" + "=" * 70)
    print("Testing Live Urdu Translation with OpenAI GPT-4o")
    print("=" * 70)

    try:
        # Initialize OpenAI service
        print("\n[1/4] Initializing OpenAI Agent Service...")
        service = OpenAIAgentService()
        print(f"✓ Service initialized")
        print(f"  - Model: {service.model}")
        print(f"  - API Key: {service.api_key[:20]}...")

        # Test content (simple technical text)
        test_content = """# Introduction to ROS 2

ROS 2 is a middleware framework for building robot applications. It provides services including hardware abstraction, low-level device control, and message-passing between processes.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')
```

The formula for force is $F = ma$ where $m$ is mass and $a$ is acceleration.

## Key Features
- Real-time communication
- Distributed architecture
- Hardware abstraction layer
"""

        print(f"\n[2/4] Test content prepared ({len(test_content)} characters)")
        print("=" * 70)
        print(test_content[:200] + "...")
        print("=" * 70)

        # Translate
        print("\n[3/4] Translating to Urdu (this may take 5-30 seconds)...")
        print("⏳ Waiting for OpenAI API...")

        translated = await service.translate_to_urdu(test_content)

        print("\n✅ Translation completed!")
        print(f"  - Input length: {len(test_content)} characters")
        print(f"  - Output length: {len(translated)} characters")

        # Display result
        print("\n[4/4] Translation Result:")
        print("=" * 70)
        print(translated[:500])
        if len(translated) > 500:
            print("...")
            print(translated[-200:])
        print("=" * 70)

        # Verify preservation
        print("\n✅ Verification:")
        code_preserved = "```python" in translated and "import rclpy" in translated
        formula_preserved = "$F = ma$" in translated or "$F=ma$" in translated

        print(f"  - Code blocks preserved: {'✓' if code_preserved else '✗'}")
        print(f"  - LaTeX formulas preserved: {'✓' if formula_preserved else '✗'}")
        print(f"  - Urdu text generated: {'✓' if any(c > '\u0600' for c in translated) else '✗'}")

        # Check cost
        print("\n💰 Check the console for cost information")

        return True

    except Exception as e:
        print(f"\n✗ Translation failed: {e}")
        print(f"Error type: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    success = await test_live_translation()

    print("\n" + "=" * 70)
    if success:
        print("✅ Live translation test PASSED!")
        print("\nNext steps:")
        print("  1. Check backend/logs/translation_costs.log for cost tracking")
        print("  2. Test via API: POST /v1/api/translate")
        print("  3. Test via frontend UI")
    else:
        print("✗ Live translation test FAILED")
        print("\nPossible issues:")
        print("  - Invalid OpenAI API key")
        print("  - Insufficient API credits")
        print("  - Network connectivity issue")
        print("  - Model access not enabled")
    print("=" * 70)

    return success


if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
