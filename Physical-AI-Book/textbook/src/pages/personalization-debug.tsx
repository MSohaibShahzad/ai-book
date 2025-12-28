/**
 * Personalization Debug Page
 *
 * Diagnostic page to help users understand why personalization button might not appear.
 */

import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { usePersonalization } from '../hooks/usePersonalization';
import { useRateLimit } from '../hooks/useRateLimit';
import Link from '@docusaurus/Link';

export default function PersonalizationDebug() {
  const {
    isAuthenticated,
    isLoading: authLoading,
    userName,
    userEmail,
    userId,
    userProfile,
    isProfileComplete
  } = useAuth();

  const { rateLimitRemaining } = usePersonalization();
  const { resetAt } = useRateLimit();

  return (
    <Layout
      title="Personalization Debug"
      description="Debug page for personalization feature"
    >
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Personalization Feature Debug</h1>
            <p>This page helps you understand why the personalization button might not be appearing.</p>

            {/* Authentication Status */}
            <div className="card margin-bottom--lg">
              <div className="card__header">
                <h3>1️⃣ Authentication Status</h3>
              </div>
              <div className="card__body">
                {authLoading ? (
                  <div className="alert alert--info">
                    Loading authentication state...
                  </div>
                ) : (
                  <>
                    <div className={`alert ${isAuthenticated ? 'alert--success' : 'alert--danger'}`}>
                      <strong>Status:</strong> {isAuthenticated ? '✅ Authenticated' : '❌ Not Authenticated'}
                    </div>

                    {isAuthenticated ? (
                      <div style={{ marginTop: '1rem' }}>
                        <p><strong>User ID:</strong> {userId || 'N/A'}</p>
                        <p><strong>Name:</strong> {userName || 'N/A'}</p>
                        <p><strong>Email:</strong> {userEmail || 'N/A'}</p>
                      </div>
                    ) : (
                      <div className="alert alert--warning" style={{ marginTop: '1rem' }}>
                        <strong>Action Required:</strong> You need to sign in first.
                        <div style={{ marginTop: '0.5rem' }}>
                          <Link to="/signup" className="button button--primary button--sm margin-right--sm">
                            Sign Up
                          </Link>
                          <Link to="/signin" className="button button--secondary button--sm">
                            Sign In
                          </Link>
                        </div>
                      </div>
                    )}
                  </>
                )}
              </div>
            </div>

            {/* Profile Completeness */}
            <div className="card margin-bottom--lg">
              <div className="card__header">
                <h3>2️⃣ Profile Completeness</h3>
              </div>
              <div className="card__body">
                {!isAuthenticated ? (
                  <div className="alert alert--secondary">
                    Sign in first to check profile status.
                  </div>
                ) : (
                  <>
                    <div className={`alert ${isProfileComplete ? 'alert--success' : 'alert--warning'}`}>
                      <strong>Status:</strong> {isProfileComplete ? '✅ Complete' : '⚠️ Incomplete'}
                    </div>

                    <div style={{ marginTop: '1rem' }}>
                      <h4>Required Fields:</h4>
                      <ul>
                        <li>
                          <strong>Software Background:</strong>{' '}
                          {userProfile?.softwareBackground ? (
                            <span className="badge badge--success">{userProfile.softwareBackground}</span>
                          ) : (
                            <span className="badge badge--danger">Missing</span>
                          )}
                        </li>
                        <li>
                          <strong>Hardware Background:</strong>{' '}
                          {userProfile?.hardwareBackground ? (
                            <span className="badge badge--success">{userProfile.hardwareBackground}</span>
                          ) : (
                            <span className="badge badge--danger">Missing</span>
                          )}
                        </li>
                        <li>
                          <strong>Interest Area:</strong>{' '}
                          {userProfile?.interestArea ? (
                            <span className="badge badge--success">{userProfile.interestArea}</span>
                          ) : (
                            <span className="badge badge--danger">Missing</span>
                          )}
                        </li>
                      </ul>
                    </div>

                    {!isProfileComplete && (
                      <div className="alert alert--warning" style={{ marginTop: '1rem' }}>
                        <strong>Action Required:</strong> Complete your profile to enable personalization.
                        <div style={{ marginTop: '0.5rem' }}>
                          <Link to="/profile/settings" className="button button--primary button--sm">
                            Complete Profile
                          </Link>
                        </div>
                      </div>
                    )}
                  </>
                )}
              </div>
            </div>

            {/* Rate Limit Status */}
            <div className="card margin-bottom--lg">
              <div className="card__header">
                <h3>3️⃣ Rate Limit Status</h3>
              </div>
              <div className="card__body">
                {!isAuthenticated ? (
                  <div className="alert alert--secondary">
                    Sign in first to check rate limit.
                  </div>
                ) : (
                  <>
                    <div className={`alert ${rateLimitRemaining > 0 ? 'alert--success' : 'alert--danger'}`}>
                      <strong>Remaining personalizations today:</strong> {rateLimitRemaining}/3
                    </div>

                    {rateLimitRemaining === 0 && resetAt && (
                      <div className="alert alert--warning" style={{ marginTop: '1rem' }}>
                        <strong>Daily limit reached.</strong> Resets at:{' '}
                        {new Date(resetAt).toLocaleString()}
                      </div>
                    )}
                  </>
                )}
              </div>
            </div>

            {/* Overall Status */}
            <div className="card margin-bottom--lg">
              <div className="card__header">
                <h3>📊 Overall Status</h3>
              </div>
              <div className="card__body">
                {!isAuthenticated ? (
                  <div className="alert alert--danger">
                    <h4>❌ Personalization Not Available</h4>
                    <p><strong>Reason:</strong> You are not signed in.</p>
                    <p><strong>Next Step:</strong> Sign in or create an account.</p>
                  </div>
                ) : !isProfileComplete ? (
                  <div className="alert alert--warning">
                    <h4>⚠️ Personalization Not Available</h4>
                    <p><strong>Reason:</strong> Your profile is incomplete.</p>
                    <p><strong>Next Step:</strong> Complete your profile with all 3 fields.</p>
                  </div>
                ) : rateLimitRemaining === 0 ? (
                  <div className="alert alert--warning">
                    <h4>⏳ Personalization Temporarily Unavailable</h4>
                    <p><strong>Reason:</strong> You've reached today's limit (3 personalizations/day).</p>
                    <p><strong>Next Step:</strong> Wait for reset at {resetAt && new Date(resetAt).toLocaleString()}</p>
                  </div>
                ) : (
                  <div className="alert alert--success">
                    <h4>✅ Personalization Available!</h4>
                    <p><strong>Status:</strong> All requirements met. You can personalize chapters.</p>
                    <p><strong>Remaining today:</strong> {rateLimitRemaining}/3 personalizations</p>
                    <div style={{ marginTop: '1rem' }}>
                      <Link
                        to="/docs/foundations-ros2/what-is-ros2-personalized-example"
                        className="button button--primary"
                      >
                        Try Personalization Now
                      </Link>
                    </div>
                  </div>
                )}
              </div>
            </div>

            {/* Instructions */}
            <div className="card">
              <div className="card__header">
                <h3>📖 How to Use Personalization</h3>
              </div>
              <div className="card__body">
                <ol>
                  <li>
                    <strong>Sign in or create an account</strong>
                    <ul>
                      <li>Go to <Link to="/signup">Sign Up</Link> if you don't have an account</li>
                      <li>Or <Link to="/signin">Sign In</Link> if you already have one</li>
                    </ul>
                  </li>
                  <li>
                    <strong>Complete your profile</strong>
                    <ul>
                      <li>Go to <Link to="/profile/settings">Profile Settings</Link></li>
                      <li>Fill in all 3 fields: Software Background, Hardware Background, Interest Area</li>
                    </ul>
                  </li>
                  <li>
                    <strong>Navigate to a personalizable chapter</strong>
                    <ul>
                      <li>Example: <Link to="/docs/foundations-ros2/what-is-ros2-personalized-example">What is ROS 2?</Link></li>
                    </ul>
                  </li>
                  <li>
                    <strong>Click "✨ Personalize for Me"</strong>
                    <ul>
                      <li>The button will appear at the top of the chapter</li>
                      <li>Wait 15-30 seconds for AI to generate personalized content</li>
                    </ul>
                  </li>
                </ol>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
