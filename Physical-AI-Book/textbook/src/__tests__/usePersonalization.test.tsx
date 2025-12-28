/**
 * Tests for usePersonalization Hook
 *
 * Tests verify that the hook:
 * 1. Calls personalize() function and updates context
 * 2. Handles API responses correctly
 * 3. Handles different error types (rate limit, timeout, validation, auth)
 * 4. Uses cached content when available
 * 5. Updates rate limit remaining
 * 6. Clears errors
 *
 * Feature: Chapter Personalization (005-chapter-personalization)
 * Task: T044
 */

import { renderHook, act, waitFor } from '@testing-library/react';
import { usePersonalization } from '../hooks/usePersonalization';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import * as personalizationService from '../lib/personalizationService';
import { PersonalizationError } from '../lib/personalizationService';
import React from 'react';

// Mock the personalization service
jest.mock('../lib/personalizationService');

const mockPersonalizeChapter = personalizationService.personalizeChapter as jest.Mock;

describe('usePersonalization', () => {
  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <PersonalizationProvider>{children}</PersonalizationProvider>
  );

  beforeEach(() => {
    jest.clearAllMocks();
    jest.spyOn(console, 'log').mockImplementation(() => {});
    jest.spyOn(console, 'error').mockImplementation(() => {});
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('Initial State', () => {
    it('should have correct initial state', () => {
      const { result } = renderHook(() => usePersonalization(), { wrapper });

      expect(result.current.isLoading).toBe(false);
      expect(result.current.error).toBeNull();
      expect(result.current.rateLimitRemaining).toBe(3);
    });

    it('should provide all required functions', () => {
      const { result } = renderHook(() => usePersonalization(), { wrapper });

      expect(typeof result.current.personalize).toBe('function');
      expect(typeof result.current.getPersonalizedContent).toBe('function');
      expect(typeof result.current.clearError).toBe('function');
    });
  });

  describe('Personalize Function', () => {
    const mockRequest = {
      chapterId: 'foundations-ros2/what-is-ros2',
      content: '# What is ROS 2?\n\nOriginal content.',
      profile: {
        softwareBackground: 'Intermediate' as const,
        hardwareBackground: 'Beginner' as const,
        interestArea: 'AI' as const,
      },
    };

    const mockResponse = {
      personalized_content: '# What is ROS 2?\n\nPersonalized for intermediate developers.',
      remaining_limit: 2,
      generation_time_ms: 15000,
    };

    it('should call API with correct parameters', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(mockPersonalizeChapter).toHaveBeenCalledWith({
        chapter_id: mockRequest.chapterId,
        chapter_content: mockRequest.content,
        user_profile: mockRequest.profile,
      });
    });

    it('should set loading to true during API call', async () => {
      let resolvePromise: (value: any) => void;
      const promise = new Promise(resolve => {
        resolvePromise = resolve;
      });

      mockPersonalizeChapter.mockReturnValue(promise);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      act(() => {
        result.current.personalize(mockRequest.chapterId, mockRequest.content, mockRequest.profile);
      });

      // Should be loading
      expect(result.current.isLoading).toBe(true);

      // Resolve the promise
      await act(async () => {
        resolvePromise!(mockResponse);
        await promise;
      });

      // Should not be loading anymore
      expect(result.current.isLoading).toBe(false);
    });

    it('should update rate limit after successful personalization', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.rateLimitRemaining).toBe(2);
    });

    it('should store personalized content in context', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      const personalizedContent = result.current.getPersonalizedContent(mockRequest.chapterId);
      expect(personalizedContent).toBe(mockResponse.personalized_content);
    });

    it('should clear error on successful personalization', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      // Set an error first
      await act(async () => {
        mockPersonalizeChapter.mockRejectedValueOnce(new Error('Previous error'));
        try {
          await result.current.personalize(
            mockRequest.chapterId,
            mockRequest.content,
            mockRequest.profile
          );
        } catch {}
      });

      expect(result.current.error).not.toBeNull();

      // Now succeed
      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toBeNull();
    });
  });

  describe('Caching Behavior', () => {
    const mockRequest = {
      chapterId: 'foundations-ros2/what-is-ros2',
      content: '# What is ROS 2?\n\nOriginal content.',
      profile: {
        softwareBackground: 'Intermediate' as const,
        hardwareBackground: 'Beginner' as const,
        interestArea: 'AI' as const,
      },
    };

    const mockResponse = {
      personalized_content: '# What is ROS 2?\n\nPersonalized content.',
      remaining_limit: 2,
      generation_time_ms: 15000,
    };

    it('should use cached content and not call API again', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      // First call
      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(mockPersonalizeChapter).toHaveBeenCalledTimes(1);

      // Second call with same chapterId
      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      // API should still only be called once
      expect(mockPersonalizeChapter).toHaveBeenCalledTimes(1);
    });

    it('should return cached content from getPersonalizedContent', async () => {
      mockPersonalizeChapter.mockResolvedValue(mockResponse);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      // Initially no content
      expect(result.current.getPersonalizedContent(mockRequest.chapterId)).toBeUndefined();

      // Personalize
      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      // Now content should be cached
      const cached = result.current.getPersonalizedContent(mockRequest.chapterId);
      expect(cached).toBe(mockResponse.personalized_content);
    });

    it('should return undefined for non-cached chapters', () => {
      const { result } = renderHook(() => usePersonalization(), { wrapper });

      expect(result.current.getPersonalizedContent('non-existent-chapter')).toBeUndefined();
    });
  });

  describe('Error Handling', () => {
    const mockRequest = {
      chapterId: 'foundations-ros2/what-is-ros2',
      content: '# What is ROS 2?\n\nOriginal content.',
      profile: {
        softwareBackground: 'Intermediate' as const,
        hardwareBackground: 'Beginner' as const,
        interestArea: 'AI' as const,
      },
    };

    it('should handle rate limit error (429)', async () => {
      const error = new PersonalizationError('Rate limit exceeded', 429, 'rate_limit_exceeded', {
        reset_at: '2025-12-25T10:00:00Z',
      });

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('Daily personalization limit exceeded');
      expect(result.current.rateLimitRemaining).toBe(0);
    });

    it('should handle timeout error (408)', async () => {
      const error = new PersonalizationError('Request timeout', 408, 'timeout');

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('timed out after 30 seconds');
    });

    it('should handle validation error (400)', async () => {
      const error = new PersonalizationError('Incomplete profile', 400, 'incomplete_profile', {
        missing_fields: ['softwareBackground', 'hardwareBackground'],
      });

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('Profile incomplete');
      expect(result.current.error).toContain('softwareBackground, hardwareBackground');
    });

    it('should handle auth error (401)', async () => {
      const error = new PersonalizationError('Unauthorized', 401, 'unauthorized');

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('You must be signed in');
    });

    it('should handle generic PersonalizationError', async () => {
      const error = new PersonalizationError('Server error', 500, 'generation_failed');

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('Failed to personalize chapter');
    });

    it('should handle generic Error', async () => {
      const error = new Error('Network error');

      mockPersonalizeChapter.mockRejectedValue(error);

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('Unexpected error');
      expect(result.current.error).toContain('Network error');
    });

    it('should handle non-Error exceptions', async () => {
      mockPersonalizeChapter.mockRejectedValue('String error');

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.error).toContain('unknown error');
    });

    it('should set isLoading to false after error', async () => {
      mockPersonalizeChapter.mockRejectedValue(new Error('Error'));

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          mockRequest.chapterId,
          mockRequest.content,
          mockRequest.profile
        );
      });

      expect(result.current.isLoading).toBe(false);
    });
  });

  describe('clearError Function', () => {
    it('should clear error when called', async () => {
      mockPersonalizeChapter.mockRejectedValue(new Error('Test error'));

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      // Create an error
      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      expect(result.current.error).not.toBeNull();

      // Clear error
      act(() => {
        result.current.clearError();
      });

      expect(result.current.error).toBeNull();
    });

    it('should not affect other state when clearing error', async () => {
      mockPersonalizeChapter.mockRejectedValue(new Error('Test error'));

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      const rateLimitBefore = result.current.rateLimitRemaining;
      const isLoadingBefore = result.current.isLoading;

      act(() => {
        result.current.clearError();
      });

      expect(result.current.rateLimitRemaining).toBe(rateLimitBefore);
      expect(result.current.isLoading).toBe(isLoadingBefore);
    });
  });

  describe('Console Logging', () => {
    it('should log success messages', async () => {
      const consoleSpy = jest.spyOn(console, 'log');

      mockPersonalizeChapter.mockResolvedValue({
        personalized_content: 'Content',
        remaining_limit: 2,
        generation_time_ms: 15000,
      });

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      expect(consoleSpy).toHaveBeenCalledWith(
        expect.stringContaining('Successfully personalized')
      );
    });

    it('should log cache hit messages', async () => {
      const consoleSpy = jest.spyOn(console, 'log');

      mockPersonalizeChapter.mockResolvedValue({
        personalized_content: 'Content',
        remaining_limit: 2,
        generation_time_ms: 15000,
      });

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      // First call
      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      consoleSpy.mockClear();

      // Second call (cached)
      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      expect(consoleSpy).toHaveBeenCalledWith(
        expect.stringContaining('Using cached content')
      );
    });

    it('should log error messages', async () => {
      const consoleSpy = jest.spyOn(console, 'error');

      mockPersonalizeChapter.mockRejectedValue(new Error('Test error'));

      const { result } = renderHook(() => usePersonalization(), { wrapper });

      await act(async () => {
        await result.current.personalize(
          'test-chapter',
          'content',
          {
            softwareBackground: 'Intermediate',
            hardwareBackground: 'Beginner',
            interestArea: 'AI',
          }
        );
      });

      expect(consoleSpy).toHaveBeenCalledWith(
        expect.stringContaining('[usePersonalization] Error:'),
        expect.any(Error)
      );
    });
  });
});
