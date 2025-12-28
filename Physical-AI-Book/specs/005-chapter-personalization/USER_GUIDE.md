# Chapter Personalization - User Guide

**Feature**: Chapter Personalization (005-chapter-personalization)
**Last Updated**: 2025-12-24

## What is Chapter Personalization?

Chapter Personalization is an AI-powered feature that adapts textbook content to your individual background and interests. When you click the "Personalize for Me" button on any chapter, our AI analyzes your profile and generates a customized version that:

- Adjusts explanations based on your software and hardware experience level
- Includes relevant examples from your area of interest
- Maintains the same learning outcomes as the original content
- Preserves all code blocks, formulas, and diagrams with 100% accuracy

## Getting Started

### Prerequisites

1. **Create an Account**: Sign up for a free account at [/signup](/signup)
2. **Complete Your Profile**: Fill out your profile with:
   - **Software Background**: Beginner, Intermediate, Advanced, or Expert
   - **Hardware Background**: None, Beginner, Intermediate, or Advanced
   - **Interest Area**: AI, Robotics, Computer Vision, Motion Control, or General

### How to Personalize a Chapter

1. **Navigate to any chapter** in the textbook
2. **Look for the "✨ Personalize for Me" button** at the top of the chapter
3. **Click the button** to start personalization (takes up to 30 seconds)
4. **View your personalized content** with the "Personalized for [Your Name]" badge

### Toggling Between Views

After personalizing a chapter, you can switch between the original and personalized versions:

1. **Use the toggle buttons** at the top: "📄 Original" or "✨ Personalized"
2. **Switch views instantly** (no re-generation needed)
3. **Scroll position is preserved** when switching views

## Rate Limits

To ensure fair usage and manage costs, personalization has a daily limit:

- **3 personalizations per day** per user
- **Resets every 24 hours** from your first personalization
- **Visual indicator** shows remaining requests (e.g., "2 personalizations remaining")
- **Countdown timer** displays time until reset

### What Happens When Limit is Reached?

When you've used all 3 daily personalizations:

- The "Personalize for Me" button shows "Daily limit reached"
- An alert displays: "🚫 Daily limit reached - Resets in [time]"
- You can still **view previously personalized chapters** using the toggle
- Limit automatically resets 24 hours after your first request

## Profile Updates

When you update your profile (software background, hardware background, or interest area):

1. **All personalized content is cleared** to ensure accuracy
2. **A notification appears**: "Profile Updated - Your personalized chapters have been cleared"
3. **Re-personalize chapters** to see content adapted to your updated profile

## Error Handling

### Timeout Errors (408)

If personalization takes longer than 30 seconds:

- **Error message**: "Personalization timed out. Please try again."
- **Retry button** allows you to try again
- **Your daily limit is decremented** (retry counts against your quota)

### Generation Failures (500)

If AI generation fails:

- **Error message**: "Unable to generate personalized content. Please try again."
- **Retry button** available
- **Your daily limit is decremented**

### Rate Limit Exceeded (429)

If you've reached your daily limit:

- **Error message**: "Daily personalization limit exceeded. Try again after reset."
- **Reset time** displayed
- **No retry button** (must wait for reset)

### Incomplete Profile (400)

If your profile is missing required fields:

- **Warning message**: "Complete your profile to enable personalization"
- **"Complete Profile" button** links to settings
- **Missing fields listed** (e.g., "Software Background, Interest Area")

## Personalization Quality

### What is Preserved?

Personalization **never modifies**:

- ✅ Code blocks and syntax
- ✅ Mathematical formulas (LaTeX)
- ✅ Diagrams and images
- ✅ YAML frontmatter
- ✅ Learning outcomes

### What is Adapted?

Personalization **customizes**:

- 📝 Explanatory text and prose
- 📝 Example descriptions
- 📝 Analogies and metaphors
- 📝 Introductory paragraphs

### Example: Beginner vs. Expert

**Original Text:**
> "ROS 2 uses DDS (Data Distribution Service) for inter-process communication."

**Beginner Personalization (softwareBackground: Beginner):**
> "ROS 2 uses a messaging system called DDS (Data Distribution Service) to help different parts of your robot talk to each other, similar to how apps on your phone communicate."

**Expert Personalization (softwareBackground: Expert):**
> "ROS 2 leverages DDS (Data Distribution Service) as its middleware layer, providing real-time publish-subscribe architecture with QoS policies for deterministic communication."

## Privacy and Data

- **Personalized content is NOT saved** to the server
- **Session-only caching** (cleared when you close the browser)
- **Your profile data** is used only for personalization
- **No tracking** of personalized content

## FAQ

### Can I personalize the same chapter multiple times?

Yes, but each personalization counts against your daily limit. If you've already personalized a chapter, it's cached for the session. To re-personalize with a different profile, update your profile first (which clears the cache).

### Can I save personalized content?

Personalized content is session-only and not saved. You can copy-paste content for personal notes, but we recommend using the toggle to reference both versions.

### Why is there a rate limit?

Rate limiting ensures:
- Fair usage across all users
- Manageable AI generation costs
- Server stability during peak times

### What if I want to disable personalization?

Simply don't click the "Personalize for Me" button. All original content remains accessible without personalization.

### Can I personalize while logged out?

No, personalization requires authentication to track rate limits and access your profile.

### How accurate is the personalization?

Personalization is powered by GPT-4o and maintains 100% accuracy for technical content (code, formulas, diagrams). Prose adaptations are reviewed for accuracy, but we recommend comparing with the original using the toggle.

## Keyboard Shortcuts

- **Alt + O**: Switch to Original view
- **Alt + P**: Switch to Personalized view
- **Alt + T**: Toggle between views

## Troubleshooting

### "Personalize for Me" button not appearing?

Check:
- ✅ Are you logged in?
- ✅ Is your profile complete?
- ✅ Does this chapter support personalization?

### Personalization is slow?

- Personalization can take up to 30 seconds for long chapters
- If it times out, try again or try a shorter chapter first

### Toggle not working?

- Refresh the page
- Clear browser cache
- Check browser console for errors

### Rate limit not resetting?

- Rate limit resets **24 hours after first request**, not at midnight
- Check the countdown timer for exact reset time
- If issue persists, contact support

## Contact Support

For issues or feedback:
- Email: support@physical-ai-textbook.com
- GitHub Issues: [github.com/your-repo/issues](https://github.com/your-repo/issues)

## Tips for Best Results

1. **Complete your profile accurately** for better personalization
2. **Use the toggle frequently** to compare original and personalized versions
3. **Update your profile** as you gain more experience
4. **Personalize strategically** - use your 3 daily requests on chapters you find challenging
5. **Read both versions** to get the full perspective

---

Enjoy your personalized learning experience! 🎓✨
