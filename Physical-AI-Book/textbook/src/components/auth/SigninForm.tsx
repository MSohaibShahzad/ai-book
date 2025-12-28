import React, { useState } from 'react';
import { signInWithEmail, type SigninData } from '../../lib/auth-client';
import styles from './AuthForms.module.css';

interface SigninFormProps {
  onSuccess?: () => void;
  onError?: (error: Error) => void;
}

export function SigninForm({ onSuccess, onError }: SigninFormProps) {
  const [formData, setFormData] = useState<SigninData>({
    email: '',
    password: '',
    rememberMe: false,
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [generalError, setGeneralError] = useState<string>('');

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(formData.email)) {
      newErrors.email = 'Please enter a valid email address';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleInputChange = (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
    const { name, value, type, checked } = e.target;
    const fieldValue = type === 'checkbox' ? checked : value;

    setFormData((prev) => ({ ...prev, [name]: fieldValue }));

    // Clear error for this field when user starts typing
    if (errors[name]) {
      setErrors((prev) => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setGeneralError('');

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);

    try {
      await signInWithEmail(formData);

      // Success - redirect to homepage
      onSuccess?.();
      window.location.href = '/';
    } catch (error: any) {
      console.error('Signin error:', error);

      // Handle specific error codes
      if (error.message?.includes('429') || error.message?.includes('Too many')) {
        setGeneralError('Too many signin attempts. Please wait 15 minutes and try again.');
      } else if (error.message?.includes('network') || error.message?.includes('fetch')) {
        setGeneralError('Network error. Please check your connection and try again.');
      } else {
        // Default to invalid credentials for any authentication failure
        // This covers wrong password, wrong email, or any auth-related error
        setGeneralError('Invalid email or password. Please try again.');
      }

      onError?.(error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form
      onSubmit={handleSubmit}
      className={styles.authForm}
      aria-label="Sign in form"
      noValidate
    >
      <h2 id="signin-heading">Sign In</h2>
      <p className={styles.subtitle} id="signin-description">
        Welcome back! Sign in to access your personalized content
      </p>

      {generalError && (
        <div
          className={styles.errorBanner}
          role="alert"
          aria-live="polite"
          aria-atomic="true"
        >
          {generalError}
        </div>
      )}

      <div className={styles.formGroup}>
        <label htmlFor="email">
          Email Address <span className={styles.required} aria-label="required">*</span>
        </label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleInputChange}
          className={errors.email ? styles.inputError : ''}
          disabled={isLoading}
          required
          aria-required="true"
          aria-invalid={errors.email ? 'true' : 'false'}
          aria-describedby={errors.email ? 'email-error' : undefined}
          autoComplete="email"
        />
        {errors.email && (
          <span className={styles.errorText} id="email-error" role="alert">
            {errors.email}
          </span>
        )}
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="password">
          Password <span className={styles.required} aria-label="required">*</span>
        </label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleInputChange}
          className={errors.password ? styles.inputError : ''}
          disabled={isLoading}
          required
          aria-required="true"
          aria-invalid={errors.password ? 'true' : 'false'}
          aria-describedby={errors.password ? 'password-error' : undefined}
          autoComplete="current-password"
        />
        {errors.password && (
          <span className={styles.errorText} id="password-error" role="alert">
            {errors.password}
          </span>
        )}
      </div>

      <div className={styles.checkboxGroup}>
        <input
          type="checkbox"
          id="rememberMe"
          name="rememberMe"
          checked={formData.rememberMe}
          onChange={handleInputChange}
          disabled={isLoading}
          aria-label="Remember me for 7 days"
        />
        <label htmlFor="rememberMe">
          Remember me for 7 days
        </label>
      </div>

      <button
        type="submit"
        className={styles.submitButton}
        disabled={isLoading}
        aria-busy={isLoading}
        aria-label={isLoading ? 'Signing in, please wait' : 'Sign in'}
      >
        {isLoading ? 'Signing In...' : 'Sign In'}
      </button>

      <p className={styles.footerText}>
        Don't have an account?{' '}
        <a href="/signup" className={styles.link}>
          Sign up here
        </a>
      </p>

      <p className={styles.footerText}>
        <a href="/forgot-password" className={styles.link}>
          Forgot your password?
        </a>
      </p>
    </form>
  );
}
