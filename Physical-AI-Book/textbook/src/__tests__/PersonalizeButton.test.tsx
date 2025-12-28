/**
 * Tests for PersonalizeButton Component
 *
 * Tests verify that the button:
 * 1. Is hidden for unauthenticated users
 * 2. Shows profile completion message when profile incomplete
 * 3. Displays loading state during personalization
 * 4. Handles errors correctly
 * 5. Shows rate limit information
 * 6. Is disabled when rate limit exceeded
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T042
 */

import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { PersonalizeButton, PersonalizeButtonProps } from '../components/PersonalizeButton';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import * as usePersonalizationHook from '../hooks/usePersonalization';
import * as useRateLimitHook from '../hooks/useRateLimit';

// Mock the hooks
jest.mock('../hooks/usePersonalization');
jest.mock('../hooks/useRateLimit');
jest.mock('../components/RateLimitDisplay', () => ({
  RateLimitDisplay: () => <div data-testid="rate-limit-display">Rate Limit Display</div>,
}));

// Mock Docusaurus Link
jest.mock('@docusaurus/Link', () => {
  return ({ to, children, ...props }: any) => (
    <a href={to} {...props}>
      {children}
    </a>
  );
});

const mockPersonalize = jest.fn();
const mockUsePersonalization = usePersonalizationHook.usePersonalization as jest.Mock;
const mockUseRateLimit = useRateLimitHook.useRateLimit as jest.Mock;

describe('PersonalizeButton', () => {
  const defaultProps: PersonalizeButtonProps = {
    chapterId: 'foundations-ros2/what-is-ros2',
    chapterContent: '# What is ROS 2?\n\nOriginal content.',
    userProfile: {
      softwareBackground: 'Intermediate',
      hardwareBackground: 'Beginner',
      interestArea: 'AI',
    },
    isAuthenticated: true,
  };

  beforeEach(() => {
    jest.clearAllMocks();

    // Default mock implementations
    mockUsePersonalization.mockReturnValue({
      personalize: mockPersonalize,
      isLoading: false,
      rateLimitRemaining: 3,
      error: null,
    });

    mockUseRateLimit.mockReturnValue({
      resetAt: null,
      hoursUntilReset: 0,
      isLoading: false,
    });
  });

  describe('Visibility Based on Authentication', () => {
    it('should hide button when user is not authenticated', () => {
      const { container } = render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} isAuthenticated={false} />
        </PersonalizationProvider>
      );

      expect(container.firstChild).toBeNull();
    });

    it('should show button when user is authenticated with complete profile', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalize for Me/i)).toBeInTheDocument();
    });
  });

  describe('Profile Completeness Validation', () => {
    it('should show completion message when profile is null', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} userProfile={null} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Complete your profile to personalize chapters/i)).toBeInTheDocument();
      expect(screen.getByText(/Complete Profile/i)).toBeInTheDocument();
    });

    it('should show completion message when softwareBackground is missing', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton
            {...defaultProps}
            userProfile={{
              softwareBackground: '' as any,
              hardwareBackground: 'Beginner',
              interestArea: 'AI',
            }}
          />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Complete your profile to personalize chapters/i)).toBeInTheDocument();
      expect(screen.getByText(/Missing: Software Background/i)).toBeInTheDocument();
    });

    it('should show completion message when hardwareBackground is missing', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton
            {...defaultProps}
            userProfile={{
              softwareBackground: 'Intermediate',
              hardwareBackground: '' as any,
              interestArea: 'AI',
            }}
          />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Missing: Hardware Background/i)).toBeInTheDocument();
    });

    it('should show completion message when interestArea is missing', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton
            {...defaultProps}
            userProfile={{
              softwareBackground: 'Intermediate',
              hardwareBackground: 'Beginner',
              interestArea: '' as any,
            }}
          />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Missing: Interest Area/i)).toBeInTheDocument();
    });

    it('should show all missing fields when profile is incomplete', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton
            {...defaultProps}
            userProfile={{
              softwareBackground: '',
              hardwareBackground: '',
              interestArea: '',
            }}
          />
        </PersonalizationProvider>
      );

      expect(
        screen.getByText(/Missing: Software Background, Hardware Background, Interest Area/i)
      ).toBeInTheDocument();
    });

    it('should link to profile settings when profile incomplete', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} userProfile={null} />
        </PersonalizationProvider>
      );

      const link = screen.getByText(/Complete Profile/i).closest('a');
      expect(link).toHaveAttribute('href', '/profile/settings');
    });
  });

  describe('Loading State', () => {
    it('should show loading state during personalization', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: true,
        rateLimitRemaining: 2,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Generating personalized content.../i)).toBeInTheDocument();
      expect(screen.getByRole('status', { hidden: true })).toBeInTheDocument();
    });

    it('should disable button during loading', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: true,
        rateLimitRemaining: 2,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      const button = screen.getByRole('button');
      expect(button).toBeDisabled();
    });

    it('should not show "Personalize for Me" text during loading', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: true,
        rateLimitRemaining: 2,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByText(/✨ Personalize for Me/i)).not.toBeInTheDocument();
    });
  });

  describe('Personalization Flow', () => {
    it('should call personalize function when button clicked', async () => {
      const user = userEvent.setup();
      mockPersonalize.mockResolvedValue(undefined);

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      const button = screen.getByText(/Personalize for Me/i);
      await user.click(button);

      await waitFor(() => {
        expect(mockPersonalize).toHaveBeenCalledWith(
          'foundations-ros2/what-is-ros2',
          '# What is ROS 2?\n\nOriginal content.',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });
    });

    it('should call onPersonalizationStart callback when clicked', async () => {
      const user = userEvent.setup();
      const onStart = jest.fn();
      mockPersonalize.mockResolvedValue(undefined);

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} onPersonalizationStart={onStart} />
        </PersonalizationProvider>
      );

      const button = screen.getByText(/Personalize for Me/i);
      await user.click(button);

      await waitFor(() => {
        expect(onStart).toHaveBeenCalledTimes(1);
      });
    });

    it('should call onPersonalizationComplete callback on success', async () => {
      const user = userEvent.setup();
      const onComplete = jest.fn();
      mockPersonalize.mockResolvedValue(undefined);

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} onPersonalizationComplete={onComplete} />
        </PersonalizationProvider>
      );

      const button = screen.getByText(/Personalize for Me/i);
      await user.click(button);

      await waitFor(() => {
        expect(onComplete).toHaveBeenCalledTimes(1);
      });
    });

    it('should call onPersonalizationError callback on error', async () => {
      const user = userEvent.setup();
      const onError = jest.fn();
      mockPersonalize.mockRejectedValue(new Error('Personalization failed'));

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} onPersonalizationError={onError} />
        </PersonalizationProvider>
      );

      const button = screen.getByText(/Personalize for Me/i);
      await user.click(button);

      await waitFor(() => {
        expect(onError).toHaveBeenCalledWith('Personalization failed');
      });
    });

    it('should handle non-Error exceptions', async () => {
      const user = userEvent.setup();
      const onError = jest.fn();
      mockPersonalize.mockRejectedValue('String error');

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} onPersonalizationError={onError} />
        </PersonalizationProvider>
      );

      const button = screen.getByText(/Personalize for Me/i);
      await user.click(button);

      await waitFor(() => {
        expect(onError).toHaveBeenCalledWith('Unknown error');
      });
    });
  });

  describe('Rate Limit Display', () => {
    it('should show remaining count when less than 3', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 2,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/\(2 remaining\)/i)).toBeInTheDocument();
    });

    it('should not show remaining count when at maximum (3)', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 3,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByText(/remaining/i)).not.toBeInTheDocument();
    });

    it('should show rate limit display component when less than 3 remaining', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 1,
        error: null,
      });

      mockUseRateLimit.mockReturnValue({
        resetAt: '2025-12-25T10:00:00Z',
        hoursUntilReset: 12,
        isLoading: false,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByTestId('rate-limit-display')).toBeInTheDocument();
    });

    it('should not show rate limit display when at maximum', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 3,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByTestId('rate-limit-display')).not.toBeInTheDocument();
    });
  });

  describe('Rate Limit Exceeded', () => {
    it('should show "Daily limit reached" when rate limit is 0', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 0,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Daily limit reached/i)).toBeInTheDocument();
    });

    it('should disable button when rate limit is 0', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 0,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      const button = screen.getByRole('button');
      expect(button).toBeDisabled();
    });

    it('should not call personalize when clicked if rate limit is 0', async () => {
      const user = userEvent.setup();
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: false,
        rateLimitRemaining: 0,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      const button = screen.getByRole('button');

      // Try to click (should not work because button is disabled)
      await user.click(button);

      expect(mockPersonalize).not.toHaveBeenCalled();
    });
  });

  describe('Accessibility', () => {
    it('should have proper button role', () => {
      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByRole('button')).toBeInTheDocument();
    });

    it('should have loading spinner with proper aria attributes', () => {
      mockUsePersonalization.mockReturnValue({
        personalize: mockPersonalize,
        isLoading: true,
        rateLimitRemaining: 2,
        error: null,
      });

      render(
        <PersonalizationProvider>
          <PersonalizeButton {...defaultProps} />
        </PersonalizationProvider>
      );

      const spinner = screen.getByRole('status', { hidden: true });
      expect(spinner).toHaveAttribute('aria-hidden', 'true');
    });
  });
});
