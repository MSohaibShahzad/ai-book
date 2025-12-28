# Feature Specification: Chapter Personalization

**Feature Branch**: `005-chapter-personalization`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Chapter Personalization — Specification"

## Clarifications

### Session 2025-12-23

- Q: How should personalization requests be rate limited to prevent abuse of the AI generation endpoint? → A: 3 personalization requests per user per day (global across all chapters). Display remaining limit on user profile page and below "Personalize for Me" button. After first use, show "Remaining limit: X" on all chapter pages. When exceeded, display "Limit exceeded."
- Q: Is the 30-second personalization target a performance goal or a hard timeout? → A: 30 seconds is a hard timeout. Requests are terminated after 30 seconds with error message and retry option.
- Q: What happens to cached personalized content when a user updates their profile mid-session? → A: Invalidate all cached personalized content when user updates profile; show prompt to re-personalize chapters.
- Q: What observability and monitoring is required for personalization requests? → A: Track essential metrics: request count, success/failure rate, average generation time, timeout count.
- Q: Are there any chapter-level access restrictions for personalization? → A: All logged-in users with complete profiles can personalize any published chapter in the textbook.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Personalized Chapter Content (Priority: P1)

A logged-in user with a complete profile (software background, hardware background, interest area) wants to read a textbook chapter adapted to their experience level. They click the "Personalize for Me" button at the beginning of the chapter, wait for the personalized version to be generated, and read content that matches their technical level.

**Why this priority**: This is the core value proposition - delivering adaptive learning content. Without this, the feature provides no value.

**Independent Test**: Can be fully tested by creating a user account with a beginner profile, navigating to any chapter, clicking "Personalize for Me", and verifying the generated content uses simpler explanations and foundational concepts appropriate for beginners.

**Acceptance Scenarios**:

1. **Given** a logged-in user with softwareBackground="Beginner" and hardwareBackground="None", **When** they click "Personalize for Me" on a chapter about ROS2 nodes, **Then** the personalized version includes beginner-friendly explanations of networking concepts and avoids assuming prior distributed systems knowledge
2. **Given** a logged-in user with softwareBackground="Advanced" and hardwareBackground="Intermediate", **When** they click "Personalize for Me" on the same chapter, **Then** the personalized version includes advanced insights about node lifecycle management and hardware integration patterns
3. **Given** a logged-in user viewing a personalized chapter, **When** they click "View Original", **Then** the system displays the original unmodified chapter content
4. **Given** a logged-in user viewing a personalized chapter, **When** they navigate to a different chapter, **Then** the new chapter displays the original version by default (personalization does not persist across chapters)

---

### User Story 2 - Toggle Between Original and Personalized Views (Priority: P2)

A logged-in user viewing a personalized chapter wants to compare it with the original content to understand what was adapted. They can toggle between "Personalized" and "Original" views using a visible toggle control.

**Why this priority**: This enables users to verify accuracy and understand adaptation choices, building trust in the personalization system. It's essential for educational integrity but secondary to the core personalization feature.

**Independent Test**: Can be fully tested by personalizing a chapter, then clicking a toggle button to switch between personalized and original views, verifying both versions render correctly and the toggle state is clearly indicated.

**Acceptance Scenarios**:

1. **Given** a user viewing a personalized chapter, **When** they click "View Original", **Then** the original chapter content replaces the personalized view and the toggle indicates "Original" mode
2. **Given** a user viewing the original chapter after personalization, **When** they click "View Personalized", **Then** the system immediately displays the previously generated personalized content without re-generating it
3. **Given** a user toggling between views, **When** they scroll to a specific section in one view and switch views, **Then** the system maintains their scroll position in the equivalent section

---

### User Story 3 - Handle Incomplete User Profiles (Priority: P3)

A logged-in user who hasn't completed their profile (missing software/hardware background) wants to personalize a chapter. The system displays a message prompting them to complete their profile and provides a link to profile settings.

**Why this priority**: This handles an edge case for users who signed up but skipped profile completion. It's lower priority because most users will complete profiles during signup (per constitution principle XV).

**Independent Test**: Can be fully tested by logging in with an account that has empty background fields, clicking "Personalize for Me", and verifying the system shows a helpful message with a link to complete the profile instead of attempting personalization.

**Acceptance Scenarios**:

1. **Given** a logged-in user with empty softwareBackground field, **When** they click "Personalize for Me", **Then** the system displays "Complete your profile to enable personalization" with a link to profile settings
2. **Given** a user who just completed their profile, **When** they return to the chapter and click "Personalize for Me", **Then** the system generates personalized content using their newly entered background information

---

### Edge Cases

- What happens when AI generation fails or times out? System displays an error message: "Unable to generate personalized content. Please try again." with a retry button and option to view the original chapter. Timeout occurs at exactly 30 seconds (hard limit).
- How does system handle very long chapters (e.g., 10,000+ words)? System may show a loading indicator with progress estimation and generate personalization in chunks if needed, but this is transparent to the user.
- What if a user clicks "Personalize for Me" while generation is already in progress? System disables the button during generation and shows a "Generating..." status to prevent duplicate requests.
- What happens when a user is not logged in? The "Personalize for Me" button is hidden or replaced with "Sign in to personalize" that redirects to the login page.
- How does the system handle chapters with complex LaTeX formulas and code blocks? All mathematical notation, code examples, and technical diagrams are preserved exactly as in the original - only prose explanations are adapted.
- What happens when a user reaches their daily personalization limit? The "Personalize for Me" button is replaced with "Limit exceeded" message. The user can still view previously generated personalized content from the current session by toggling views. The limit resets 24 hours after their first request of the day.
- What happens when a user updates their profile during an active session? All cached personalized content is invalidated. The system displays a notification: "Profile updated. Re-personalize your chapters for adapted content." Chapters revert to showing original content by default.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a "Personalize for Me" button at the beginning of each published textbook chapter for logged-in users with complete profiles (no chapter-level access restrictions)
- **FR-002**: System MUST send the original chapter content and user's profile metadata (softwareBackground, hardwareBackground, interestArea) to the backend when personalization is requested
- **FR-003**: System MUST generate personalized chapter content using AI that adapts explanations to the user's experience level while preserving all code examples, formulas, and learning outcomes
- **FR-004**: System MUST display personalized content in a way that clearly indicates it's adapted (e.g., badge or header stating "Personalized for [User Name]")
- **FR-005**: System MUST provide a toggle control allowing users to switch between personalized and original views of the chapter
- **FR-006**: System MUST preserve the original markdown structure, frontmatter, code blocks, LaTeX formulas, and diagrams in the personalized version
- **FR-007**: System MUST cache generated personalized content for the duration of the user session to avoid redundant generation when toggling views
- **FR-022**: System MUST invalidate all cached personalized content when the user updates their profile (softwareBackground, hardwareBackground, or interestArea)
- **FR-023**: System MUST display a notification prompting the user to re-personalize chapters after profile updates (e.g., "Profile updated. Re-personalize your chapters for adapted content.")
- **FR-008**: System MUST NOT persist personalized content across sessions or modify the original textbook markdown files
- **FR-009**: System MUST prompt users with incomplete profiles to complete their background information before enabling personalization
- **FR-010**: System MUST display appropriate error messages when personalization generation fails, with options to retry or view the original content
- **FR-021**: System MUST terminate personalization requests that exceed 30 seconds and display a timeout error message with retry option
- **FR-011**: System MUST hide or disable the "Personalize for Me" button for users who are not logged in
- **FR-012**: System MUST respect the user's authentication state and session when making personalization requests to the backend
- **FR-013**: System MUST adapt content tone and depth based on user's softwareBackground level: Beginner (foundational explanations, no assumed knowledge), Intermediate (moderate complexity, some prior knowledge assumed), Advanced (technical depth, advanced concepts), Expert (concise, cutting-edge insights)
- **FR-014**: System MUST adapt hardware-related explanations based on user's hardwareBackground: None (extra context on physical concepts), Beginner (basic hardware explanations), Intermediate (assumes familiarity with common platforms), Advanced (assumes hands-on experience)
- **FR-015**: System MUST emphasize concepts related to the user's interestArea (AI, Robotics, Computer Vision, Motion Control, General) when personalizing content
- **FR-016**: System MUST enforce a rate limit of 3 personalization requests per user per day (calculated on a rolling 24-hour basis from first request)
- **FR-017**: System MUST display the user's remaining personalization limit count on their profile page
- **FR-018**: System MUST display the remaining personalization limit count below the "Personalize for Me" button after the user has made at least one personalization request
- **FR-019**: System MUST replace the "Personalize for Me" button with "Limit exceeded" message when the user has exhausted their daily quota
- **FR-020**: System MUST track personalization request counts per user and reset the count after 24 hours from the first request of the day
- **FR-024**: System MUST log and track the following metrics for operational monitoring: total personalization request count, success rate, failure rate, average generation time, and timeout count
- **FR-025**: System MUST make metrics accessible for monitoring dashboards to verify the 90% success rate target (SC-007)

### Key Entities

- **Personalized Chapter**: A dynamically generated version of a textbook chapter adapted to a specific user's background. Contains adapted prose explanations but identical code, formulas, and learning outcomes. Exists only in memory during the user session (not persisted to database or files).
- **User Profile Metadata**: The user's softwareBackground (Beginner/Intermediate/Advanced/Expert), hardwareBackground (None/Beginner/Intermediate/Advanced), and interestArea (AI/Robotics/Computer Vision/Motion Control/General) used as input to the personalization algorithm.
- **Original Chapter Content**: The unmodified textbook chapter markdown content, which serves as the source for personalization and remains accessible via toggle.
- **Personalization Quota**: Tracks the number of personalization requests made by each user within a rolling 24-hour window. Contains: userId, requestCount (0-3), firstRequestTimestamp. Resets when 24 hours elapse from the first request.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Logged-in users with complete profiles can successfully generate personalized content for any textbook chapter and view the adapted version within 30 seconds of clicking "Personalize for Me" (hard timeout - requests exceeding 30 seconds are terminated with error message)
- **SC-002**: Users can toggle between personalized and original views of a chapter instantly (< 1 second) without re-generation
- **SC-003**: Personalized content maintains 100% accuracy of technical terms, code examples, mathematical formulas, and learning outcomes compared to the original chapter
- **SC-004**: Users can identify which view (personalized vs. original) they are currently reading through clear visual indicators
- **SC-005**: Users with incomplete profiles receive clear guidance to complete their profile before personalization is enabled
- **SC-006**: Personalization failures (timeout, API errors) provide users with a clear path to retry or continue with original content
- **SC-007**: 90% of personalization requests complete successfully without errors in production environment

## Assumptions

- User profile data (softwareBackground, hardwareBackground, interestArea) is already collected during signup (per constitution principle XV) and stored in the user database
- The AI personalization backend endpoint exists and accepts chapter content + user metadata as input, returning adapted markdown
- Personalization quality and accuracy are validated by the AI system according to constitution principles XXIII-XXV
- All original textbook chapters are properly formatted markdown with YAML frontmatter
- The Docusaurus frontend has access to chapter markdown content for sending to the personalization API
- User sessions are managed via Better-Auth session cookies (per existing authentication implementation)

## Dependencies

- Better-Auth authentication system (feature 003-auth-signup-signin) must be fully operational
- User profile data fields (softwareBackground, hardwareBackground, interestArea) must exist in the database schema
- Backend API endpoint for AI-based chapter personalization must be implemented
- Frontend must have access to session data to determine if user is logged in and has a complete profile

## Out of Scope

- Pre-generating personalized content for all users and all chapters (personalization is on-demand only)
- Persisting personalized content to database or allowing users to save personalized versions permanently
- Personalizing exercises, code examples, or mathematical formulas (only prose explanations are adapted)
- Providing personalization for logged-out users or users without complete profiles
- Multi-language personalization (this feature only handles adaptation within English content, separate from Urdu translation feature)
- Allowing users to customize the personalization algorithm or provide feedback on quality
- Personalizing based on learning history, quiz scores, or other behavioral data (only profile metadata is used)
