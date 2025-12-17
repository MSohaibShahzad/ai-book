import React, { useState } from 'react';
import { useSession, signOut } from '../../lib/auth-client';
import styles from './UserMenu.module.css';

export function UserMenu() {
  const { data: session, isPending, error } = useSession();
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [isSigningOut, setIsSigningOut] = useState(false);

  // Check for session expiration
  React.useEffect(() => {
    if (error && (error.message?.includes('401') || error.message?.includes('expired'))) {
      // Session expired - show message and redirect to signin
      const shouldRedirect = window.confirm(
        'Your session has expired. Please sign in again to continue.'
      );
      if (shouldRedirect) {
        window.location.href = '/signin';
      }
    }
  }, [error]);

  const handleSignOut = async () => {
    setIsSigningOut(true);
    try {
      await signOut();
      window.location.href = '/';
    } catch (error) {
      console.error('Sign out error:', error);
      setIsSigningOut(false);
    }
  };

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  // Close menu when clicking outside
  React.useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const target = event.target as HTMLElement;
      if (isMenuOpen && !target.closest(`.${styles.userMenu}`)) {
        setIsMenuOpen(false);
      }
    };

    document.addEventListener('click', handleClickOutside);
    return () => document.removeEventListener('click', handleClickOutside);
  }, [isMenuOpen]);

  // Loading state
  if (isPending) {
    return (
      <div className={styles.userMenu}>
        <div className={styles.skeleton}></div>
      </div>
    );
  }

  // Error state (treat as not authenticated)
  if (error) {
    return (
      <div className={styles.userMenu}>
        <a href="/signin" className={styles.signInLink}>
          Sign In
        </a>
        <a href="/signup" className={styles.signUpButton}>
          Sign Up
        </a>
      </div>
    );
  }

  // Not authenticated
  if (!session) {
    return (
      <div className={styles.userMenu}>
        <a href="/signin" className={styles.signInLink}>
          Sign In
        </a>
        <a href="/signup" className={styles.signUpButton}>
          Sign Up
        </a>
      </div>
    );
  }

  // Authenticated - show user menu
  return (
    <div className={styles.userMenu}>
      <button
        className={styles.userButton}
        onClick={toggleMenu}
        aria-expanded={isMenuOpen}
        aria-haspopup="true"
      >
        <span className={styles.userAvatar}>
          {session.user.name.charAt(0).toUpperCase()}
        </span>
        <span className={styles.userName}>{session.user.name}</span>
        <svg
          className={styles.chevron}
          width="12"
          height="12"
          viewBox="0 0 12 12"
          fill="currentColor"
        >
          <path d="M2 4l4 4 4-4H2z" />
        </svg>
      </button>

      {isMenuOpen && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownHeader}>
            <div className={styles.userInfo}>
              <div className={styles.userNameLarge}>{session.user.name}</div>
              <div className={styles.userEmail}>{session.user.email}</div>
            </div>
          </div>

          <div className={styles.dropdownDivider}></div>

          <a href="/profile" className={styles.dropdownItem}>
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 8a3 3 0 100-6 3 3 0 000 6zm0 2c-2.67 0-8 1.34-8 4v1h16v-1c0-2.66-5.33-4-8-4z" />
            </svg>
            Profile Settings
          </a>

          <div className={styles.dropdownDivider}></div>

          <button
            onClick={handleSignOut}
            className={styles.dropdownItem}
            disabled={isSigningOut}
          >
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M3 2v12h6v-2H5V4h4V2H3zm9 3l-4 4 4 4v-3h4V8h-4V5z" />
            </svg>
            {isSigningOut ? 'Signing Out...' : 'Sign Out'}
          </button>
        </div>
      )}
    </div>
  );
}
