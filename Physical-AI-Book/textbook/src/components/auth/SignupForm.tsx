import React, { useState } from 'react';
import { signUpWithBackground, type SignupData } from '../../lib/auth-client';
import styles from './AuthForms.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
  onError?: (error: Error) => void;
}

export function SignupForm({ onSuccess, onError }: SignupFormProps) {
  const [formData, setFormData] = useState<SignupData>({
    email: '',
    password: '',
    name: '',
    softwareBackground: 'Beginner',
    hardwareBackground: 'None',
    interestArea: 'AI',
  });

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [generalError, setGeneralError] = useState<string>('');
  const [passwordStrength, setPasswordStrength] = useState<{
    score: number;
    label: string;
    color: string;
  }>({ score: 0, label: '', color: '' });

  const validateEmail = (email: string): boolean => {
    // RFC 5322 simplified email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const calculatePasswordStrength = (password: string) => {
    if (!password) {
      setPasswordStrength({ score: 0, label: '', color: '' });
      return;
    }

    let score = 0;

    // Length check
    if (password.length >= 8) score += 1;
    if (password.length >= 12) score += 1;

    // Complexity checks
    if (/[a-z]/.test(password)) score += 1; // lowercase
    if (/[A-Z]/.test(password)) score += 1; // uppercase
    if (/[0-9]/.test(password)) score += 1; // numbers
    if (/[^a-zA-Z0-9]/.test(password)) score += 1; // special chars

    // Determine strength
    let label = '';
    let color = '';
    if (score <= 2) {
      label = 'Weak';
      color = '#ef4444'; // red
    } else if (score <= 4) {
      label = 'Fair';
      color = '#f59e0b'; // orange
    } else if (score <= 5) {
      label = 'Good';
      color = '#10b981'; // green
    } else {
      label = 'Strong';
      color = '#059669'; // dark green
    }

    setPasswordStrength({ score, label, color });
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
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters long';
    }

    if (!formData.name) {
      newErrors.name = 'Name is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleInputChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));

    // Calculate password strength
    if (name === 'password') {
      calculatePasswordStrength(value);
    }

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
      await signUpWithBackground(formData);

      // Success - redirect to homepage or show success message
      onSuccess?.();
      window.location.href = '/';
    } catch (error: any) {
      console.error('Signup error:', error);

      // Handle specific error codes
      if (error.message?.includes('429') || error.message?.includes('Too many')) {
        setGeneralError('Too many signup attempts. Please wait 15 minutes and try again.');
      } else if (error.message?.includes('already exists') || error.message?.includes('already registered')) {
        setGeneralError('This email is already registered. Please try signing in instead.');
      } else if (error.message?.includes('400') || error.message?.includes('validation')) {
        setGeneralError('Please check your input and try again.');
      } else if (error.message?.includes('network') || error.message?.includes('fetch')) {
        setGeneralError('Network error. Please check your connection and try again.');
      } else {
        setGeneralError('An error occurred during signup. Please try again later.');
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
      aria-label="Sign up form"
      noValidate
    >
      <h2 id="signup-heading">Create Your Account</h2>
      <p className={styles.subtitle} id="signup-description">
        Join to access personalized content based on your background
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
        <label htmlFor="name">
          Full Name <span className={styles.required} aria-label="required">*</span>
        </label>
        <input
          type="text"
          id="name"
          name="name"
          value={formData.name}
          onChange={handleInputChange}
          className={errors.name ? styles.inputError : ''}
          disabled={isLoading}
          required
          aria-required="true"
          aria-invalid={errors.name ? 'true' : 'false'}
          aria-describedby={errors.name ? 'name-error' : undefined}
          autoComplete="name"
        />
        {errors.name && (
          <span className={styles.errorText} id="name-error" role="alert">
            {errors.name}
          </span>
        )}
      </div>

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
          aria-describedby={errors.email ? 'email-error' : 'email-help'}
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
          minLength={8}
          required
          aria-required="true"
          aria-invalid={errors.password ? 'true' : 'false'}
          aria-describedby={errors.password ? 'password-error' : 'password-help'}
          autoComplete="new-password"
        />
        {errors.password && (
          <span className={styles.errorText} id="password-error" role="alert">
            {errors.password}
          </span>
        )}
        {passwordStrength.label && (
          <div
            className={styles.passwordStrength}
            role="status"
            aria-live="polite"
            aria-label={`Password strength: ${passwordStrength.label}`}
          >
            <div className={styles.strengthBar} aria-hidden="true">
              <div
                className={styles.strengthProgress}
                style={{
                  width: `${(passwordStrength.score / 6) * 100}%`,
                  backgroundColor: passwordStrength.color,
                }}
              />
            </div>
            <span style={{ color: passwordStrength.color, fontSize: '0.875rem', fontWeight: 500 }}>
              {passwordStrength.label}
            </span>
          </div>
        )}
        <span className={styles.helpText} id="password-help">
          Use 8+ characters with a mix of letters, numbers & symbols
        </span>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="softwareBackground">
          Software Programming Background
        </label>
        <select
          id="softwareBackground"
          name="softwareBackground"
          value={formData.softwareBackground}
          onChange={handleInputChange}
          disabled={isLoading}
        >
          <option value="Beginner">Beginner - New to programming</option>
          <option value="Intermediate">Intermediate - Some experience</option>
          <option value="Advanced">Advanced - Regular developer</option>
          <option value="Expert">Expert - Professional developer</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="hardwareBackground">
          Hardware/Robotics Background
        </label>
        <select
          id="hardwareBackground"
          name="hardwareBackground"
          value={formData.hardwareBackground}
          onChange={handleInputChange}
          disabled={isLoading}
        >
          <option value="None">None - No experience</option>
          <option value="Beginner">Beginner - Basic hobby projects</option>
          <option value="Intermediate">Intermediate - Regular projects</option>
          <option value="Advanced">Advanced - Professional experience</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="interestArea">
          Primary Interest Area
        </label>
        <select
          id="interestArea"
          name="interestArea"
          value={formData.interestArea}
          onChange={handleInputChange}
          disabled={isLoading}
        >
          <option value="AI">Artificial Intelligence</option>
          <option value="Robotics">Robotics & Mechatronics</option>
          <option value="Computer Vision">Computer Vision</option>
          <option value="Motion Control">Motion Control & Planning</option>
          <option value="General">General - All topics</option>
        </select>
      </div>

      <button
        type="submit"
        className={styles.submitButton}
        disabled={isLoading}
        aria-busy={isLoading}
        aria-label={isLoading ? 'Creating account, please wait' : 'Sign up'}
      >
        {isLoading ? 'Creating Account...' : 'Sign Up'}
      </button>

      <p className={styles.footerText}>
        Already have an account?{' '}
        <a href="/signin" className={styles.link}>
          Sign in here
        </a>
      </p>
    </form>
  );
}
