/**
 * Tests for ViewToggle Component
 *
 * Tests verify that the component:
 * 1. Toggles between original and personalized views
 * 2. Updates currentView state correctly
 * 3. Preserves scroll position when switching views
 * 4. Shows badge when in personalized view
 * 5. Hides when no personalized content exists
 * 6. Calls onViewChange callback
 * 7. Has proper accessibility attributes
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T045
 */

import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { ViewToggle } from '../components/ViewToggle';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import * as PersonalizationContext from '../contexts/PersonalizationContext';

const mockSetCurrentView = jest.fn();
const mockUsePersonalizationContext = jest.spyOn(PersonalizationContext, 'usePersonalizationContext');

describe('ViewToggle', () => {
  const defaultProps = {
    userName: 'John Doe',
    hasPersonalizedContent: true,
  };

  beforeEach(() => {
    jest.clearAllMocks();

    // Mock window.scrollY and window.scrollTo
    Object.defineProperty(window, 'scrollY', {
      writable: true,
      value: 0,
    });

    window.scrollTo = jest.fn();

    // Default mock for context
    mockUsePersonalizationContext.mockReturnValue({
      currentView: 'original' as PersonalizationContext.ViewMode,
      personalizedContent: new Map(),
      setCurrentView: mockSetCurrentView,
      addPersonalizedContent: jest.fn(),
      clearPersonalizedContent: jest.fn(),
      rateLimitRemaining: 3,
      setRateLimitRemaining: jest.fn(),
    });
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('Visibility', () => {
    it('should render when hasPersonalizedContent is true', () => {
      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Original/i)).toBeInTheDocument();
      expect(screen.getByText(/Personalized/i)).toBeInTheDocument();
    });

    it('should hide when hasPersonalizedContent is false', () => {
      const { container } = render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} hasPersonalizedContent={false} />
        </PersonalizationProvider>
      );

      expect(container.firstChild).toBeNull();
    });
  });

  describe('Toggle Functionality', () => {
    it('should show both Original and Personalized buttons', () => {
      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByRole('button', { name: /Original/i })).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /Personalized/i })).toBeInTheDocument();
    });

    it('should call setCurrentView when Original button clicked', async () => {
      const user = userEvent.setup();

      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      await user.click(originalButton);

      expect(mockSetCurrentView).toHaveBeenCalledWith('original');
    });

    it('should call setCurrentView when Personalized button clicked', async () => {
      const user = userEvent.setup();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });
      await user.click(personalizedButton);

      expect(mockSetCurrentView).toHaveBeenCalledWith('personalized');
    });

    it('should not call setCurrentView when clicking currently active button', async () => {
      const user = userEvent.setup();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      await user.click(originalButton);

      // Should not be called because currentView is already 'original'
      expect(mockSetCurrentView).not.toHaveBeenCalled();
    });
  });

  describe('Active State Indication', () => {
    it('should show Original button as active when currentView is original', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'original' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      expect(originalButton).toHaveAttribute('aria-pressed', 'true');
      expect(originalButton).toHaveClass('button--primary');
    });

    it('should show Personalized button as active when currentView is personalized', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });
      expect(personalizedButton).toHaveAttribute('aria-pressed', 'true');
      expect(personalizedButton).toHaveClass('button--primary');
    });

    it('should show Original button as inactive when currentView is personalized', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      expect(originalButton).toHaveAttribute('aria-pressed', 'false');
      expect(originalButton).toHaveClass('button--outline');
    });
  });

  describe('Badge Display', () => {
    it('should show badge when currentView is personalized', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalized for John Doe/i)).toBeInTheDocument();
    });

    it('should not show badge when currentView is original', () => {
      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(screen.queryByText(/Personalized for/i)).not.toBeInTheDocument();
    });

    it('should show "Personalized for you" when userName is not provided', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} userName={undefined} />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalized for you/i)).toBeInTheDocument();
    });

    it('should include userName in badge when provided', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} userName="Jane Smith" />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalized for Jane Smith/i)).toBeInTheDocument();
    });
  });

  describe('Scroll Preservation', () => {
    it('should preserve scroll position when switching views', async () => {
      const user = userEvent.setup();

      // Set initial scroll position
      Object.defineProperty(window, 'scrollY', { value: 500, writable: true });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });
      await user.click(personalizedButton);

      // Wait for requestAnimationFrame to fire
      await waitFor(() => {
        expect(window.scrollTo).toHaveBeenCalledWith({
          top: 500,
          behavior: 'auto',
        });
      });
    });

    it('should add scroll event listener on mount', () => {
      const addEventListenerSpy = jest.spyOn(window, 'addEventListener');

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      expect(addEventListenerSpy).toHaveBeenCalledWith(
        'scroll',
        expect.any(Function),
        { passive: true }
      );
    });

    it('should remove scroll event listener on unmount', () => {
      const removeEventListenerSpy = jest.spyOn(window, 'removeEventListener');

      const { unmount } = render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      unmount();

      expect(removeEventListenerSpy).toHaveBeenCalledWith(
        'scroll',
        expect.any(Function)
      );
    });
  });

  describe('Callback Handling', () => {
    it('should call onViewChange callback when view changes', async () => {
      const user = userEvent.setup();
      const onViewChange = jest.fn();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} onViewChange={onViewChange} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });
      await user.click(personalizedButton);

      expect(onViewChange).toHaveBeenCalledWith('personalized');
    });

    it('should not call onViewChange when clicking currently active button', async () => {
      const user = userEvent.setup();
      const onViewChange = jest.fn();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} onViewChange={onViewChange} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      await user.click(originalButton);

      expect(onViewChange).not.toHaveBeenCalled();
    });

    it('should not crash when onViewChange is not provided', async () => {
      const user = userEvent.setup();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });

      // Should not throw
      await expect(user.click(personalizedButton)).resolves.not.toThrow();
    });
  });

  describe('Accessibility', () => {
    it('should have proper aria-label on button group', () => {
      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const buttonGroup = screen.getByRole('group');
      expect(buttonGroup).toHaveAttribute(
        'aria-label',
        'Toggle between original and personalized views'
      );
    });

    it('should set aria-pressed correctly on buttons', () => {
      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });

      expect(originalButton).toHaveAttribute('aria-pressed', 'true');
      expect(personalizedButton).toHaveAttribute('aria-pressed', 'false');
    });

    it('should update aria-pressed when view changes', async () => {
      const user = userEvent.setup();

      const { rerender } = render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      // Switch to personalized view
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      rerender(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const originalButton = screen.getByRole('button', { name: /Original/i });
      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });

      expect(originalButton).toHaveAttribute('aria-pressed', 'false');
      expect(personalizedButton).toHaveAttribute('aria-pressed', 'true');
    });
  });

  describe('Edge Cases', () => {
    it('should handle rapid toggle clicks gracefully', async () => {
      const user = userEvent.setup();

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} />
        </PersonalizationProvider>
      );

      const personalizedButton = screen.getByRole('button', { name: /Personalized/i });

      // Click multiple times rapidly
      await user.click(personalizedButton);
      await user.click(personalizedButton);
      await user.click(personalizedButton);

      // Should only call setCurrentView once (first click)
      expect(mockSetCurrentView).toHaveBeenCalledTimes(1);
    });

    it('should handle empty userName string', () => {
      mockUsePersonalizationContext.mockReturnValue({
        currentView: 'personalized' as PersonalizationContext.ViewMode,
        personalizedContent: new Map(),
        setCurrentView: mockSetCurrentView,
        addPersonalizedContent: jest.fn(),
        clearPersonalizedContent: jest.fn(),
        rateLimitRemaining: 3,
        setRateLimitRemaining: jest.fn(),
      });

      render(
        <PersonalizationProvider>
          <ViewToggle {...defaultProps} userName="" />
        </PersonalizationProvider>
      );

      expect(screen.getByText(/Personalized for you/i)).toBeInTheDocument();
    });
  });
});
