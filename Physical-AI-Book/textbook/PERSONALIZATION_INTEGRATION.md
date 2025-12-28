# Chapter Personalization Integration Guide

This guide shows how to add personalization capabilities to textbook chapters.

## Quick Start

### Option 1: Using PersonalizableChapter Component (Recommended)

The easiest way to add personalization to a chapter is using the `PersonalizableChapter` wrapper component.

#### Step 1: Convert your .md file to .mdx

Rename your chapter file from `.md` to `.mdx`:
```bash
mv docs/foundations-ros2/01-what-is-ros2.md docs/foundations-ros2/01-what-is-ros2.mdx
```

#### Step 2: Import and wrap your content

Add the import at the top of your .mdx file and wrap your content:

```mdx
---
title: "Chapter 1: What is ROS 2?"
slug: what-is-ros2
sidebar_label: "1. What is ROS 2?"
---

import PersonalizableChapter from '@site/src/components/PersonalizableChapter';

<PersonalizableChapter chapterId="foundations-ros2/what-is-ros2">

# Chapter 1: What is ROS 2?

## Introduction

Your chapter content here...

</PersonalizableChapter>
```

That's it! The component automatically handles:
- ✅ Authentication state
- ✅ Profile fetching
- ✅ Personalize button display
- ✅ View toggle
- ✅ Error handling
- ✅ Rate limiting

### Option 2: Manual Integration (Advanced)

For more control, you can manually integrate the personalization components:

```mdx
---
title: "Chapter 1: What is ROS 2?"
slug: what-is-ros2
---

import { PersonalizeButton } from '@site/src/components/PersonalizeButton';
import { PersonalizedChapter } from '@site/src/components/PersonalizedChapter';
import { useAuth } from '@site/src/hooks/useAuth';
import { usePersonalizationContext } from '@site/src/contexts/PersonalizationContext';

export function ChapterWithPersonalization() {
  const { isAuthenticated, userProfile, userName } = useAuth();
  const { currentView } = usePersonalizationContext();

  const chapterContent = `
# Chapter 1: What is ROS 2?

Your full chapter markdown content here...
  `;

  return (
    <div>
      {isAuthenticated && (
        <PersonalizeButton
          chapterId="foundations-ros2/what-is-ros2"
          chapterContent={chapterContent}
          userProfile={userProfile}
          isAuthenticated={isAuthenticated}
        />
      )}

      <PersonalizedChapter
        chapterId="foundations-ros2/what-is-ros2"
        originalContent={chapterContent}
        userName={userName}
      />
    </div>
  );
}

<ChapterWithPersonalization />
```

## Chapter ID Convention

The `chapterId` should follow this pattern:
```
{module-name}/{chapter-slug}
```

Examples:
- `foundations-ros2/what-is-ros2`
- `digital-twin/gazebo-physics-simulation`
- `ai-robot-brain/isaac-sim-photorealism`

## How It Works

### 1. User Flow

1. **Unauthenticated User**: Sees normal chapter content
2. **Authenticated User (Incomplete Profile)**: Sees message to complete profile
3. **Authenticated User (Complete Profile)**:
   - Sees "Personalize for Me" button
   - Clicks button → AI personalizes content based on their profile
   - Can toggle between original and personalized views
   - Rate limited to 3 personalizations per day

### 2. Components Overview

#### PersonalizableChapter
- High-level wrapper that handles everything
- Use this unless you need custom behavior

#### PersonalizeButton
- Shows "Personalize for Me" button
- Handles loading state and errors
- Shows remaining rate limit
- Displays profile completion message

#### PersonalizedChapter
- Renders personalized or original content
- Integrates ViewToggle for switching
- Shows loading spinner during generation
- Handles errors with retry option

#### useAuth Hook
- Provides authentication state
- Fetches user profile from Better-Auth
- Checks if profile is complete

### 3. Context Providers

The app is already wrapped with required providers in `src/theme/Root.js`:

```jsx
<TranslationProvider>
  <PersonalizationProvider>
    {children}
    <ProfileUpdateNotification />
  </PersonalizationProvider>
</TranslationProvider>
```

## Testing Locally

### 1. Start the Auth Server

```bash
cd backend
python -m uvicorn main:app --reload --port 3001
```

### 2. Start Docusaurus

```bash
cd textbook
npm start
```

### 3. Test the Flow

1. Sign up at `http://localhost:3000/signup`
2. Complete your profile (Software Background, Hardware Background, Interest Area)
3. Navigate to a personalized chapter
4. Click "Personalize for Me"
5. Wait for AI generation (~15-20 seconds)
6. Toggle between Original and Personalized views

## Troubleshooting

### "Complete your profile" message won't go away

Make sure all three profile fields are filled:
- Software Background (Beginner/Intermediate/Advanced/Expert)
- Hardware Background (None/Beginner/Intermediate/Advanced)
- Interest Area (AI/Robotics/Computer Vision/Motion Control/General)

### Button says "Daily limit reached"

Each user gets 3 personalizations per day (rolling 24-hour window).
Wait for the reset time shown in the notification.

### Personalization times out

Chapters longer than ~3000 words may timeout (30 second limit).
Consider splitting very long chapters into smaller sections.

### Profile updates don't trigger cache invalidation

The `ProfileUpdateNotification` component should appear when you update your profile.
Click "Clear Personalized Content" to invalidate the cache.

## Rate Limits

- **Quota**: 3 personalizations per user per day
- **Window**: Rolling 24-hour window from first request
- **Timeout**: 30 seconds per personalization
- **Counted On**: Success AND timeout (to prevent spam)

## API Endpoints

Personalization uses these backend endpoints:

- `POST /api/personalization/personalize` - Generate personalized content
- `GET /api/personalization/quota` - Check remaining quota
- `DELETE /api/personalization/cache` - Clear cached content
- `GET /api/personalization/metrics` - View system metrics

## Best Practices

1. **Chapter Length**: Keep chapters under 3000 words for reliable personalization
2. **Chapter IDs**: Use consistent naming: `{module}/{slug}`
3. **Testing**: Always test with different user profiles (Beginner vs Expert)
4. **Error Handling**: The components handle errors automatically
5. **Performance**: Personalized content is cached in-session (until page refresh)

## Migration Checklist

To add personalization to an existing chapter:

- [ ] Rename `.md` to `.mdx`
- [ ] Add `import PersonalizableChapter` statement
- [ ] Wrap content in `<PersonalizableChapter chapterId="...">` tags
- [ ] Test locally with authentication
- [ ] Verify personalization works for different profiles
- [ ] Check that toggle works correctly
- [ ] Test rate limiting behavior

## Examples

See these chapters for reference implementations:

- `textbook/docs/foundations-ros2/01-what-is-ros2.mdx` (if migrated)
- `textbook/src/components/PersonalizableChapter.tsx` (component source)

## Support

For issues or questions:
- Check `/backend/personalization/README.md` for backend details
- Review test files in `textbook/src/__tests__/` for component behavior
- See `specs/005-chapter-personalization/` for full specification
