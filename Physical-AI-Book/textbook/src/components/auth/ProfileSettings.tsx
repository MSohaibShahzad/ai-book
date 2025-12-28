import React, { useState, useEffect } from 'react';
import { useSession, updateProfile, type ProfileUpdateData } from '../../lib/auth-client';
import styles from './AuthForms.module.css';

export function ProfileSettings() {
  const { data: session, isPending, error, refetch } = useSession();
  const [formData, setFormData] = useState<ProfileUpdateData>({
    softwareBackground: undefined,
    hardwareBackground: undefined,
    interestArea: undefined,
  });

  const [isLoading, setIsLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState<string>('');
  const [errorMessage, setErrorMessage] = useState<string>('');

  // Populate form with current values when session loads
  useEffect(() => {
    if (session?.user) {
      setFormData({
        softwareBackground: session.user.softwareBackground as any,
        hardwareBackground: session.user.hardwareBackground as any,
        interestArea: session.user.interestArea as any,
      });
    }
  }, [session]);

  const handleInputChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setSuccessMessage('');
    setErrorMessage('');
    setIsLoading(true);

    try {
      await updateProfile(formData);

      // Refetch session to get updated data
      await refetch();

      setSuccessMessage('Profile updated successfully! Your personalized content will reflect these changes.');
    } catch (error: any) {
      console.error('Profile update error:', error);
      setErrorMessage(error.message || 'Failed to update profile. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  // Loading state
  if (isPending) {
    return (
      <div className={styles.authForm}>
        <h2>Loading...</h2>
      </div>
    );
  }

  // Error or not authenticated
  if (error || !session) {
    return (
      <div className={styles.authForm}>
        <h2>Not Authenticated</h2>
        <p>Please <a href="/signin">sign in</a> to access your profile settings.</p>
      </div>
    );
  }

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2>Profile Settings</h2>
      <p className={styles.subtitle}>
        Update your technical background to get more personalized content
      </p>

      {successMessage && (
        <div className={styles.successBanner} role="alert">
          {successMessage}
        </div>
      )}

      {errorMessage && (
        <div className={styles.errorBanner} role="alert">
          {errorMessage}
        </div>
      )}

      <div className={styles.formGroup}>
        <label htmlFor="name">
          Name
        </label>
        <input
          type="text"
          id="name"
          value={session.user.name}
          disabled
          style={{ opacity: 0.6 }}
        />
        <span className={styles.helpText}>Name cannot be changed</span>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="email">
          Email Address
        </label>
        <input
          type="email"
          id="email"
          value={session.user.email}
          disabled
          style={{ opacity: 0.6 }}
        />
        <span className={styles.helpText}>Email cannot be changed</span>
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
      >
        {isLoading ? 'Updating...' : 'Save Changes'}
      </button>
    </form>
  );
}
