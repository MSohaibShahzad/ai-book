#!/usr/bin/env python3
"""
Quick password reset script for Better-Auth users.
Hashes a new password using the same bcrypt algorithm that Better-Auth uses.
"""

import sys
import psycopg2
import os
from dotenv import load_dotenv
import bcrypt

load_dotenv()


def reset_password(email: str, new_password: str):
    """Reset password for a user."""
    try:
        # Connect to database
        conn = psycopg2.connect(os.getenv('DATABASE_URL'))
        cur = conn.cursor()

        # Check if user exists
        cur.execute('SELECT id, email, name FROM "user" WHERE email = %s', (email,))
        user = cur.fetchone()

        if not user:
            print(f"❌ User with email '{email}' not found")
            cur.close()
            conn.close()
            return False

        print(f"\n✓ Found user: {user[2]} ({user[1]})")

        # Hash the new password using bcrypt (same as Better-Auth)
        password_bytes = new_password.encode('utf-8')
        salt = bcrypt.gensalt(rounds=10)
        hashed = bcrypt.hashpw(password_bytes, salt).decode('utf-8')

        print(f"✓ Generated password hash")

        # Update the password
        cur.execute('''
            UPDATE "user"
            SET password = %s, "updatedAt" = NOW()
            WHERE email = %s
        ''', (hashed, email))

        conn.commit()

        print(f"✅ Password updated successfully for {email}")
        print(f"\nYou can now sign in with:")
        print(f"  Email: {email}")
        print(f"  Password: {new_password}")

        cur.close()
        conn.close()
        return True

    except Exception as e:
        print(f"❌ Error: {e}")
        return False


def main():
    if len(sys.argv) != 3:
        print("Usage: python reset_password.py <email> <new_password>")
        print("\nExample:")
        print("  python reset_password.py sohaibshahzad30@gmail.com MyNewPassword123")
        sys.exit(1)

    email = sys.argv[1]
    new_password = sys.argv[2]

    print("=" * 60)
    print("Better-Auth Password Reset")
    print("=" * 60)

    success = reset_password(email, new_password)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
