import React from 'react';
import Layout from '@theme/Layout';
import { SignupForm } from '../components/auth/SignupForm';

export default function SignupPage() {
  return (
    <Layout
      title="Sign Up"
      description="Create your account to access personalized Physical-AI content"
    >
      <div style={{ padding: '2rem 0' }}>
        <SignupForm />
      </div>
    </Layout>
  );
}
