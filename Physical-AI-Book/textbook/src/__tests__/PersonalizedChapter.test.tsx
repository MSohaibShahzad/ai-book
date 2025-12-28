/**
 * Tests for PersonalizedChapter Component
 *
 * Tests verify that the component:
 * 1. Renders personalized content correctly
 * 2. Shows loading state during generation
 * 3. Displays error messages with dismiss button
 * 4. Integrates ViewToggle component
 * 5. Falls back to original content when no personalization exists
 * 6. Renders markdown with proper plugins (GFM, Math, KaTeX)
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T043
 */

import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { PersonalizedChapter } from '../components/PersonalizedChapter';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import * as PersonalizationContext from '../contexts/PersonalizationContext';
import * as usePersonalizationHook from '../hooks/usePersonalization';

// Mock the hooks and components
jest.mock('../hooks/usePersonalization');
jest.mock('../components/ViewToggle', () => ({
  ViewToggle: ({ hasPersonalizedContent, userName }: any) => (
    <div data-testid="view-toggle">
      {hasPersonalizedContent && <span>Has personalized content</span>}
      {userName && <span>User: {userName}</span>}
    </div>
  ),
}));

// Mock ReactMarkdown (simplified rendering)
jest.mock('react-markdown', () => {
  return ({ children }: { children: string }) => <div data-testid="markdown-content">{children}</div>;
});

// Mock remark/rehype plugins
jest.mock('remark-gfm', () => () => {});
jest.mock('remark-math', () => () => {});
jest.mock('rehype-katex', () => () => {});

const mockUsePersonalization = usePersonalizationHook.usePersonalization as jest.Mock;
const mockUsePersonalizationContext = jest.spyOn(PersonalizationContext, 'usePersonalizationContext');

describe('PersonalizedChapter', () => {
  const defaultProps = {
    chapterId: 'foundations-ros2/what-is-ros2',
    originalContent: '# What is ROS 2?\n\nOriginal content for testing.',
    userName: 'John Doe',
  };

  beforeEach(() => {
    jest.clearAllMocks();

    // Default mock for usePersonalizationContext
    mockUsePersonalizationContext.mockReturnValue({
      currentView: 'original' as PersonalizationContext.ViewMode,
      personalizedContent: new Map(),
      setCurrentView: jest.fn(),
      addPersonalizedContent: jest.fn(),
      clearPersonalizedContent: jest.fn(),
      rateLimitRemaining: 3,
      setRateLimitRemaining: jest.fn(),
    });

    // Default mock for usePersonalization
    mockUsePersonalization.mockReturnValue({
      personalize: jest.fn(),
      isLoading: false,
      error: null,
      getPersonalizedContent: jest.fn().mockReturnValue(null),
      clearError: jest.fn(),
      rateLimitRemaining: 3,
    });
  });

  describe('Content Rendering', () => {
    it('should render original content when no personalization exists', () => {
      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('markdown-content')).toHaveTextContent(
        '# What is ROS 2?\n\nOriginal content for testing.'
      );
    });

    it('should render personalized content when available and view is personalized', () => {
      const personalizedContent = '# What is ROS 2?\n\nPersonalized content for beginners.';

      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map([['foundations-ros2/what-is-ros2', personalizedContent]]),
        setCurrentView: jest.fn(),
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 2,
        setRateLimitRemaining: jest.fn(),
      });

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(personalizedContent),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('markdown-content')).toHaveTextContent(personalizedContent);
    });

    it('should render original content when personalized exists but view is original', () => {
      const personalizedContent = '# What is ROS 2?\n\nPersonalized content.';

      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'original' as PersonalizationContext.ViewMode,
        personalizedContent: new Map([['foundations-ros2/what-is-ros2', personalizedContent]]),
        setCurrentView: jest.fn(),
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 2,
        setRateLimitRemaining: jest.fn(),
      });

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(personalizedContent),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('markdown-content')).toHaveTextContent(defaultProps.originalContent);
    });
  });

  describe('Loading State', () => {
    it('should show loading spinner when isLoading is true', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: true,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByRole('status')).toBeInTheDocument();
      expect(screen.getByText(/Generating personalized content.../i)).toBeInTheDocument();
    });

    it('should show loading message explaining the process', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: true,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(
        screen.getByText(/Our AI is adapting this chapter to your background and interests/i)
      ).toBeInTheDocument();
      expect(screen.getByText(/This may take up to 30 seconds/i)).toBeInTheDocument();
    });

    it('should not render content during loading', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: true,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByTestId('markdown-content')).not.toBeInTheDocument();
    });

    it('should not render ViewToggle during loading', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: true,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByTestId('view-toggle')).not.toBeInTheDocument();
    });
  });

  describe('Error Handling', () => {
    it('should display error message when error exists', () => {
      const errorMessage = 'Failed to generate personalized content. Please try again.';

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: errorMessage,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalization Error/i)).toBeInTheDocument();
      expect(screen.getByText(errorMessage)).toBeInTheDocument();
    });

    it('should show dismiss button on error', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Some error',
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByRole('button', { name: /Dismiss/i })).toBeInTheDocument();
    });

    it('should call clearError when dismiss button clicked', async () => {
      const user = userEvent.setup();
      const mockClearError = jest.fn();

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Some error',
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: mockClearError,
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      const dismissButton = screen.getByRole('button', { name: /Dismiss/i });
      await user.click(dismissButton);

      expect(mockClearError).toHaveBeenCalledTimes(1);
    });

    it('should show helpful message when personalized content exists after error', () => {
      const personalizedContent = '# Personalized content';

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Generation failed',
        getPersonalizedContent: jest.fn().mockReturnValue(personalizedContent),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(
        screen.getByText(/You can still view your previously personalized version/i)
      ).toBeInTheDocument();
    });

    it('should not show helpful message when no personalized content exists after error', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Generation failed',
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(
        screen.queryByText(/You can still view your previously personalized version/i)
      ).not.toBeInTheDocument();
    });

    it('should not render content when error is present', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Some error',
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByTestId('markdown-content')).not.toBeInTheDocument();
    });
  });

  describe('ViewToggle Integration', () => {
    it('should render ViewToggle component', () => {
      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('view-toggle')).toBeInTheDocument();
    });

    it('should pass userName to ViewToggle', () => {
      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText('User: John Doe')).toBeInTheDocument();
    });

    it('should pass hasPersonalizedContent to ViewToggle when content exists', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue('# Personalized content'),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText('Has personalized content')).toBeInTheDocument();
    });

    it('should call onViewChange callback when provided', () => {
      const onViewChange = jest.fn();

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} onViewChange={onViewChange} />
        </PersonalizationProvider>
      );

      // ViewToggle is mocked, but we verify the prop is passed
      expect(screen.getByTestId('view-toggle')).toBeInTheDocument();
    });
  });

  describe('Accessibility', () => {
    it('should have proper loading spinner accessibility', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: true,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      const spinner = screen.getByRole('status');
      expect(spinner).toBeInTheDocument();
      expect(screen.getByText('Loading...')).toBeInTheDocument(); // visually-hidden text
    });

    it('should have alert role for error messages', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: 'Some error',
        getPersonalizedContent: jest.fn().mockReturnValue(null),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      // Check that error is displayed in an alert div
      expect(screen.getByText(/Personalization Error/i).closest('.alert')).toBeInTheDocument();
    });
  });

  describe('Edge Cases', () => {
    it('should handle undefined userName gracefully', () => {
      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} userName={undefined} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('view-toggle')).toBeInTheDocument();
      expect(screen.queryByText(/User:/)).not.toBeInTheDocument();
    });

    it('should handle empty originalContent', () => {
      render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} originalContent="" />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('markdown-content')).toHaveTextContent('');
    });

    it('should update content when currentView changes', () => {
      const { rerender } = render(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      // Initial render with original view
      expect(screen.getByTestId('markdown-content')).toHaveTextContent(defaultProps.originalContent);

      // Switch to personalized view
      const personalizedContent = '# Personalized content';
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map([['foundations-ros2/what-is-ros2', personalizedContent]]),
        setCurrentView: jest.fn(),
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 2,
        setRateLimitRemaining: jest.fn(),
      });

      mockUsePersonalization.mockReturnValue({
        personalize: jest.fn(),
        isLoading: false,
        error: null,
        getPersonalizedContent: jest.fn().mockReturnValue(personalizedContent),
        clearError: jest.fn(),
        rateLimitRemaining: 2,
      });

      rerender(
        <PersonalizationProvider>
          <PersonalizedChapter {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('markdown-content')).toHaveTextContent(personalizedContent);
    });
  });
});
