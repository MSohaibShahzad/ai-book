const { Pool } = require('pg');
const fs = require('fs');
const path = require('path');
require('dotenv').config();

async function cleanAndMigrate() {
  const pool = new Pool({
    connectionString: process.env.DATABASE_URL,
  });

  try {
    console.log('🧹 Cleaning existing Better Auth tables...');

    // Drop all Better Auth tables in correct order (respecting foreign keys)
    const dropTables = [
      'DROP TABLE IF EXISTS "verification" CASCADE',
      'DROP TABLE IF EXISTS "session" CASCADE',
      'DROP TABLE IF EXISTS "account" CASCADE',
      'DROP TABLE IF EXISTS "user" CASCADE',
    ];

    for (const dropSQL of dropTables) {
      try {
        await pool.query(dropSQL);
        console.log(`✓ ${dropSQL}`);
      } catch (error) {
        console.log(`⚠ ${dropSQL} - ${error.message}`);
      }
    }

    console.log('\n📋 Creating fresh Better Auth tables...');

    // Create tables with correct schema
    const createSQL = `
      -- User table
      CREATE TABLE "user" (
        "id" text PRIMARY KEY NOT NULL,
        "name" text NOT NULL,
        "email" text NOT NULL UNIQUE,
        "emailVerified" boolean NOT NULL DEFAULT false,
        "image" text,
        "createdAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "updatedAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "softwareBackground" text,
        "hardwareBackground" text,
        "interestArea" text
      );

      -- Session table
      CREATE TABLE "session" (
        "id" text PRIMARY KEY NOT NULL,
        "userId" text NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
        "expiresAt" timestamptz NOT NULL,
        "token" text NOT NULL UNIQUE,
        "createdAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "updatedAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "ipAddress" text,
        "userAgent" text
      );

      CREATE INDEX "session_userId_idx" ON "session"("userId");

      -- Account table
      CREATE TABLE "account" (
        "id" text PRIMARY KEY NOT NULL,
        "userId" text NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
        "accountId" text NOT NULL,
        "providerId" text NOT NULL,
        "accessToken" text,
        "refreshToken" text,
        "idToken" text,
        "accessTokenExpiresAt" timestamptz,
        "refreshTokenExpiresAt" timestamptz,
        "scope" text,
        "password" text,
        "createdAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "updatedAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP
      );

      CREATE INDEX "account_userId_idx" ON "account"("userId");

      -- Verification table
      CREATE TABLE "verification" (
        "id" text PRIMARY KEY NOT NULL,
        "identifier" text NOT NULL,
        "value" text NOT NULL,
        "expiresAt" timestamptz NOT NULL,
        "createdAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP,
        "updatedAt" timestamptz NOT NULL DEFAULT CURRENT_TIMESTAMP
      );
    `;

    await pool.query(createSQL);
    console.log('✅ All tables created successfully!');

    console.log('\n🎉 Database is ready for Better Auth!');
    console.log('\nTables created:');
    console.log('  ✓ user (with softwareBackground, hardwareBackground, interestArea)');
    console.log('  ✓ session');
    console.log('  ✓ account');
    console.log('  ✓ verification');

  } catch (error) {
    console.error('❌ Migration failed:', error.message);
    process.exit(1);
  } finally {
    await pool.end();
  }
}

cleanAndMigrate();
